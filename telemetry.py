"""Read-only command telemetry, restricted to loopback (use an SSH tunnel).

The HTTP thread serves cached JSON only. It has no motor output or controller
callback. A slow viewer never holds a lock while network I/O is in progress.
"""
from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import os
import socket
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import ProxyHandler, build_opener
import uuid

from robot_config import LEGS, RobotConfig, SafetyError, finite_number, integer

PROTOCOL_VERSION = 1
DEFAULT_PORT = 8765
MODES = ("DRY_RUN", "HARDWARE")


def encode_json(value):
    return json.dumps(value, ensure_ascii=True, allow_nan=False, separators=(",", ":")).encode("utf-8")


def command_record(controller, elapsed):
    """Copy the most recently accepted output, never a future servo target."""
    frame = controller.frame
    return {
        "time": float(elapsed), "pose": controller.last_pose.copy(),
        "feet": {leg: frame.feet[leg].tolist() for leg in LEGS},
        "body_shift": frame.body_shift.tolist(), "swing": list(frame.swing_legs),
        "state": controller.state, "velocity": frame.effective_velocity.tolist(),
        "margin": float(frame.support_margin_m), "tracking": float(controller.tracking_error_deg),
        "support_model_valid": controller.posture == "STAND" and controller.transition is None,
    }


def validate_record(config, record):
    """Treat even local/tunnel data as untrusted before using it in the plot."""
    try:
        def vector(value, size, label, limit):
            if not isinstance(value, list) or len(value) != size:
                raise SafetyError(f"{label}: expected {size} coordinates")
            return [finite_number(v, label, -limit, limit) for v in value]

        if not isinstance(record, dict) or not isinstance(record["pose"], dict):
            raise SafetyError("Invalid telemetry record")
        if not isinstance(record["feet"], dict) or set(record["feet"]) != set(LEGS):
            raise SafetyError("Telemetry must contain four feet")
        swing = record["swing"]
        if not isinstance(swing, list) or len(swing) > 1 or any(leg not in LEGS for leg in swing):
            raise SafetyError("Invalid crawl swing legs")
        state = record["state"]
        if not isinstance(state, str) or not 1 <= len(state) <= 40 or not state.replace("_", "").isalnum():
            raise SafetyError("Invalid telemetry state")
        if type(record["support_model_valid"]) is not bool:
            raise SafetyError("Missing support-model status")
        return {
            "time": finite_number(record["time"], "telemetry time", 0, 1e9),
            "pose": config.validate_pose(record["pose"]),
            "feet": {leg: vector(record["feet"][leg], 3, leg, 2) for leg in LEGS},
            "body_shift": vector(record["body_shift"], 3, "body shift", 1),
            "swing": tuple(swing), "state": state,
            "velocity": vector(record["velocity"], 3, "velocity", 10),
            "margin": finite_number(record["margin"], "support margin", -2, 2),
            "tracking": finite_number(record["tracking"], "tracking", 0, 180),
            "support_model_valid": record["support_model_valid"],
        }
    except (KeyError, TypeError, AttributeError) as exc:
        raise SafetyError(f"Malformed telemetry: {exc}") from exc


class _LocalServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    allow_reuse_address = os.name != "nt"

    def server_bind(self):
        # Windows SO_REUSEADDR can bind an already occupied telemetry port.
        if os.name == "nt":
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

    def get_request(self):
        connection, address = super().get_request()
        connection.settimeout(.5)
        return connection, address


class TelemetryServer:
    def __init__(self, config, *, port=DEFAULT_PORT, mode="DRY_RUN"):
        self.port = integer(port, "telemetry port", 0, 65535)
        if mode not in MODES:
            raise SafetyError("Unknown telemetry mode")
        self.run_id = uuid.uuid4().hex
        self.mode = mode
        self._metadata = encode_json({"version": PROTOCOL_VERSION, "run_id": self.run_id,
                                      "mode": mode, "config": config.data, "poses": config.poses})
        self._lock = threading.Lock()
        self._sequence = 0
        self._state = {"version": PROTOCOL_VERSION, "run_id": self.run_id, "mode": mode,
                       "sequence": 0, "active": False, "record": None}
        self._state_bytes = encode_json(self._state)
        self._server = None
        self._thread = None

    def open(self):
        if self._server is not None:
            raise SafetyError("Telemetry server is already open")
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass  # no terminal traffic in the control loop

            def do_GET(self):
                if self.path == "/metadata":
                    body = owner._metadata
                elif self.path == "/state":
                    with owner._lock:
                        body = owner._state_bytes
                else:
                    self.send_error(404)
                    return
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.end_headers()
                    self.wfile.write(body)
                except (OSError, TimeoutError):
                    pass

            def do_POST(self):
                self.send_error(405, "Read-only telemetry; no control endpoint")

            do_PUT = do_PATCH = do_DELETE = do_POST

        # Deliberately no configurable public host and no motor control routes.
        server = _LocalServer(("127.0.0.1", self.port), Handler)
        self._server = server
        self.port = server.server_address[1]
        self._thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .05},
                                        name="sm5-telemetry", daemon=True)
        try:
            self._thread.start()
        except BaseException:
            server.server_close()
            self._server = None
            self._thread = None
            raise

    def publish(self, controller, elapsed):
        # JSON creation happens outside the short cache lock, never on a socket.
        self._sequence += 1
        state = {"version": PROTOCOL_VERSION, "run_id": self.run_id, "mode": self.mode,
                 "sequence": self._sequence, "active": True,
                 "record": command_record(controller, elapsed)}
        body = encode_json(state)
        with self._lock:
            self._state, self._state_bytes = state, body

    def close(self):
        if self._server is None:
            return
        with self._lock:
            self._state = {**self._state, "active": False}
            self._state_bytes = encode_json(self._state)
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=.6)
        self._server = None
        self._thread = None


class TelemetryClient:
    """Read-only polling outside the GUI thread, with explicit stale detection."""
    def __init__(self, *, port=DEFAULT_PORT, timeout=.5, stale_after=1.5, clock=time.monotonic):
        self.port = integer(port, "viewer port", 1, 65535)
        self.timeout = finite_number(timeout, "network timeout", .05, 2)
        self.stale_after = finite_number(stale_after, "stale interval", .2, 10)
        self.clock = clock
        # A proxy would route loopback telemetry outside the laptop unnecessarily.
        self._opener = build_opener(ProxyHandler({}))
        self.config = None
        self.mode = None
        self.run_id = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._record = None
        self._sequence = -1
        self._progress_at = None
        self._active = False
        self._error = ""

    def _read(self, route, limit):
        with self._opener.open(f"http://127.0.0.1:{self.port}{route}", timeout=self.timeout) as response:
            body = response.read(limit + 1)
        if len(body) > limit:
            raise SafetyError("Telemetry response exceeds its size limit")
        try:
            value = json.loads(body)
        except (ValueError, UnicodeError, RecursionError) as exc:
            raise SafetyError("Invalid telemetry JSON") from exc
        if not isinstance(value, dict):
            raise SafetyError("Expected a telemetry object")
        return value

    def connect(self):
        metadata = self._read("/metadata", 256*1024)
        try:
            if type(metadata["version"]) is not int or metadata["version"] != PROTOCOL_VERSION:
                raise SafetyError("Unsupported telemetry protocol")
            if metadata["mode"] not in MODES:
                raise SafetyError("Unknown telemetry mode")
            run_id = metadata["run_id"]
            if not isinstance(run_id, str) or len(run_id) != 32 or any(c not in "0123456789abcdef" for c in run_id):
                raise SafetyError("Invalid telemetry session")
            self.config = RobotConfig(metadata["config"], metadata["poses"])
            self.run_id, self.mode = run_id, metadata["mode"]
        except (KeyError, TypeError, AttributeError) as exc:
            raise SafetyError(f"Malformed telemetry metadata: {exc}") from exc
        self.poll_once()
        return self.config

    def poll_once(self):
        try:
            state = self._read("/state", 64*1024)
            if (type(state.get("version")) is not int or state["version"] != PROTOCOL_VERSION
                    or state.get("run_id") != self.run_id or state.get("mode") != self.mode):
                raise SafetyError("Sesiune schimbată: redeschide vizualizatorul")
            sequence = integer(state["sequence"], "sequence", 0, 10**12)
            if type(state["active"]) is not bool:
                raise SafetyError("Invalid output status")
            record = validate_record(self.config, state["record"]) if state["record"] is not None else None
            if state["active"] and record is None:
                raise SafetyError("Active output without a command record")
            with self._lock:
                if sequence < self._sequence:
                    raise SafetyError("Telemetry sequence moved backwards")
                if sequence != self._sequence:
                    self._record = record
                    self._sequence = sequence
                    self._progress_at = self.clock()
                self._active = state["active"]
                self._error = ""
            return True
        except (OSError, URLError, HTTPError, ValueError, TypeError, KeyError, AttributeError) as exc:
            with self._lock:
                self._error = str(exc)[:150]
            return False

    def start(self):
        if self.config is None:
            raise SafetyError("Connect the viewer before starting polling")
        if self._thread is not None:
            raise SafetyError("Viewer is already polling")
        self._stop.clear()
        def poll():
            while not self._stop.wait(.1):
                self.poll_once()
        self._thread = threading.Thread(target=poll, name="sm5-viewer-poll", daemon=True)
        self._thread.start()

    def sample(self):
        """Return (last record, short Romanian status, stream is fresh)."""
        with self._lock:
            record, error, active, progress = self._record, self._error, self._active, self._progress_at
        mode = "DRY RUN — FĂRĂ MOTOARE" if self.mode == "DRY_RUN" else "HARDWARE — COMENZI"
        if error:
            return record, f"{mode}  |  CONEXIUNE ÎNTRERUPTĂ / DATE INVALIDE — imagine înghețată", False
        if record is None:
            return None, f"{mode}  |  AȘTEPT COMENZI — poziția desenată este doar referința STAND", False
        if not active:
            return record, f"{mode}  |  FLUX ÎNCHIS — nu presupune că robotul este în siguranță", False
        age = self.clock() - progress
        if age > self.stale_after:
            return record, f"{mode}  |  DATE VECHI ({age:.1f} s) — imagine înghețată", False
        return record, f"{mode}  |  LIVE · date noi acum {age:.1f} s · fără feedback măsurat", True

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.timeout + .3)
            self._thread = None
