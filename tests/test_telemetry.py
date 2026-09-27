from contextlib import redirect_stdout, redirect_stderr
import copy
import io
from pathlib import Path
import signal
import socket
import subprocess
import sys
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

from controller import MotionController
from robot_config import SafetyError, load_config
import runner_gait_ikfin as runner
from telemetry import TelemetryClient, TelemetryServer, command_record, validate_record

ROOT = Path(__file__).resolve().parents[1]


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.controller = MotionController(self.config)
        self.server = TelemetryServer(self.config, port=0)
        self.server.open()
        self.addCleanup(self.server.close)
        self.server.publish(self.controller, 0.)
        self.opener = build_opener(ProxyHandler({}))

    def client(self, **kwargs):
        client = TelemetryClient(port=self.server.port, **kwargs)
        self.addCleanup(client.close)
        client.connect()
        return client

    def test_loopback_only_and_exact_config_and_output_round_trip(self):
        self.assertEqual(self.server._server.server_address[0], "127.0.0.1")
        client = self.client()
        record, label, fresh = client.sample()
        self.assertTrue(fresh)
        self.assertIn("FĂRĂ MOTOARE", label)
        self.assertEqual(client.config.data, self.config.data)
        self.assertEqual(client.config.poses, self.config.poses)
        self.assertEqual(record["pose"], self.controller.last_pose)
        self.assertFalse(client.config.hardware["calibration_verified_on_robot"])
        self.assertEqual(client.mode, "DRY_RUN")

    def test_post_put_delete_and_control_routes_cannot_change_commands(self):
        original = self.controller.last_pose.copy()
        for method, route, code in (("POST", "/state", 405), ("PUT", "/metadata", 405),
                                     ("DELETE", "/state", 405), ("GET", "/control", 404)):
            request = Request(f"http://127.0.0.1:{self.server.port}{route}",
                              data=b'{"command":"arm"}' if method != "GET" else None, method=method)
            with self.assertRaises(HTTPError) as raised:
                self.opener.open(request, timeout=1)
            self.assertEqual(raised.exception.code, code)
            raised.exception.close()
        self.assertEqual(self.client().sample()[0]["pose"], original)
        self.assertEqual(self.controller.last_pose, original)

    def test_cached_frame_cannot_be_changed_by_later_controller_mutation(self):
        expected = self.controller.last_pose.copy()
        self.controller.last_pose["FL_up"] += 1
        self.controller.frame.feet["FL"][0] += .01
        record = self.client().sample()[0]
        self.assertEqual(record["pose"], expected)
        self.assertNotEqual(record["feet"]["FL"][0], self.controller.frame.feet["FL"][0])

    def test_incomplete_http_client_does_not_block_output_publication(self):
        with socket.create_connection(("127.0.0.1", self.server.port), timeout=1) as slow:
            slow.sendall(b"GET /state HTTP/1.0\r\n")
            self.controller.update(.02)
            self.server.publish(self.controller, .02)
            client = self.client()
            self.assertEqual(client.sample()[0]["time"], .02)

    def test_frozen_sequence_is_stale_even_when_http_keeps_responding(self):
        clock = [10.]
        client = self.client(clock=lambda: clock[0])
        self.assertTrue(client.sample()[2])
        clock[0] += 2.
        self.assertTrue(client.poll_once())
        record, label, fresh = client.sample()
        self.assertFalse(fresh)
        self.assertIn("DATE VECHI", label)
        self.server.publish(self.controller, 2.)
        self.assertTrue(client.poll_once())
        self.assertTrue(client.sample()[2])

    def test_disconnect_retains_last_frame_but_never_claims_live(self):
        client = self.client()
        before = client.sample()[0]
        self.server.close()
        self.assertFalse(client.poll_once())
        after, label, fresh = client.sample()
        self.assertIs(before, after)
        self.assertFalse(fresh)
        self.assertIn("ÎNTRERUPTĂ", label)

    def test_new_session_is_not_silently_drawn_with_old_calibration(self):
        client = self.client()
        before = client.sample()[0]
        port = self.server.port
        self.server.close()
        replacement = TelemetryServer(self.config, port=port, mode="HARDWARE")
        replacement.open()
        self.addCleanup(replacement.close)
        replacement.publish(self.controller, .1)
        self.assertFalse(client.poll_once())
        self.assertIs(client.sample()[0], before)
        self.assertFalse(client.sample()[2])
        self.assertEqual(client.mode, "DRY_RUN")

    def test_pose_transitions_do_not_claim_planner_contacts_are_valid(self):
        self.controller.request_pose("CROUCH_READY")
        self.controller.update(.02)
        self.server.publish(self.controller, .02)
        record = self.client().sample()[0]
        self.assertEqual(record["state"], "POSE_TRANSITION")
        self.assertFalse(record["support_model_valid"])

    def test_malformed_nonfinite_and_out_of_limits_frames_are_rejected(self):
        original = command_record(self.controller, 0.)
        variants = []
        for value in (float("nan"), float("inf"), 999, True, "90"):
            record = copy.deepcopy(original)
            record["pose"]["FL_up"] = value
            variants.append(record)
        record = copy.deepcopy(original); del record["pose"]["FL_up"]; variants.append(record)
        record = copy.deepcopy(original); record["body_shift"] = [0, 0]; variants.append(record)
        record = copy.deepcopy(original); record["feet"]["FL"][0] = float("nan"); variants.append(record)
        record = copy.deepcopy(original); record["swing"] = ["FL", "RR"]; variants.append(record)
        record = copy.deepcopy(original); record["support_model_valid"] = "yes"; variants.append(record)
        for record in variants:
            with self.subTest(record=record), self.assertRaises(SafetyError):
                validate_record(self.config, record)

    def test_invalid_port_and_metadata_fail_closed(self):
        for port in (-1, 65536, True):
            with self.assertRaises(SafetyError):
                TelemetryServer(self.config, port=port)
        self.server._metadata = b'{"version":true}'
        with self.assertRaises(SafetyError):
            self.client()

    def test_bad_frame_retains_last_good_frame_and_shows_error(self):
        client = self.client()
        previous = client.sample()[0]
        with self.server._lock:
            self.server._state_bytes = b'{"not_a_valid_frame":true}'
        self.assertFalse(client.poll_once())
        record, label, fresh = client.sample()
        self.assertIs(record, previous)
        self.assertFalse(fresh)
        self.assertIn("DATE INVALIDE", label)

    def test_waiting_server_is_not_labelled_as_live(self):
        waiting = TelemetryServer(self.config, port=0)
        waiting.open()
        self.addCleanup(waiting.close)
        client = TelemetryClient(port=waiting.port)
        self.addCleanup(client.close)
        client.connect()
        record, label, fresh = client.sample()
        self.assertIsNone(record)
        self.assertFalse(fresh)
        self.assertIn("AȘTEPT COMENZI", label)


class TelemetryRunnerTests(unittest.TestCase):
    @unittest.skipUnless(hasattr(signal, "SIGHUP"), "SIGHUP is a Unix terminal signal")
    def test_ssh_hangup_requests_output_cleanup_and_restores_signal(self):
        previous = signal.getsignal(signal.SIGHUP)
        def hangup(*_):
            self.assertIs(signal.getsignal(signal.SIGHUP), runner._interrupt)
            signal.raise_signal(signal.SIGHUP)
        with patch.object(runner.MotionController, "update", side_effect=hangup), \
             patch.object(runner.DryRunOutput, "close") as close, \
             redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
            runner.run_realtime(load_config(), (0,0,0), .1, "STAND")
        close.assert_called_once()
        self.assertEqual(signal.getsignal(signal.SIGHUP), previous)

    def test_hardware_flags_still_block_before_network_or_motor_open(self):
        with patch.object(runner.TelemetryServer, "open", side_effect=AssertionError("no network")), \
             patch.object(runner.HardwareOutput, "open", side_effect=AssertionError("no motors")), \
             redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(runner.main(["--hardware", "--initial-pose", "STAND", "--telemetry"]), 2)

    def test_telemetry_enables_realtime_dry_run_without_changing_hardware_default(self):
        with patch.object(runner, "run_realtime") as run:
            self.assertEqual(runner.main(["--telemetry", "--vx", "0", "--duration", ".1"]), 0)
        self.assertFalse(run.call_args.kwargs["hardware"])
        self.assertEqual(run.call_args.kwargs["telemetry_port"], 8765)
        self.assertEqual(run.call_args.args[1], (0., 0., 0.))

    def test_occupied_port_fails_before_motor_open(self):
        config = load_config()
        server = TelemetryServer(config, port=0)
        server.open()
        self.addCleanup(server.close)
        with patch.object(runner.HardwareOutput, "open") as arm, self.assertRaises(OSError):
            runner.run_realtime(config, (0,0,0), .1, "STAND", hardware=True, telemetry_port=server.port)
        arm.assert_not_called()

    def test_motor_shutdown_precedes_network_cleanup_and_failed_frame_is_not_published(self):
        events = []
        class Output:
            cleanup_errors = []
            def __init__(self, _): pass
            def open(self, _): events.append("output_open")
            def write(self, *_):
                events.append("write_failed")
                raise SafetyError("injected failure")
            def close(self): events.append("output_close")
        class Stream:
            port = 8765
            def __init__(self, *_, **__): pass
            def open(self): events.append("stream_open")
            def publish(self, *_): events.append("publish")
            def close(self): events.append("stream_close")
        previous = signal.getsignal(signal.SIGTERM)
        with patch.object(runner, "DryRunOutput", Output), patch.object(runner, "TelemetryServer", Stream), \
             redirect_stdout(io.StringIO()), self.assertRaises(SafetyError):
            runner.run_realtime(load_config(), (0,0,0), .1, "STAND", telemetry_port=8765)
        self.assertEqual(events, ["stream_open", "output_open", "publish", "write_failed", "output_close", "stream_close"])
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)

    def test_telemetry_and_viewer_imports_cannot_touch_hardware(self):
        script = '''
import builtins
original = builtins.__import__
def checked(name, *args, **kwargs):
    if name.split('.')[0] in {'adafruit_servokit','smbus2','board','busio','adafruit_pca9685'}:
        raise AssertionError('hardware import attempted: '+name)
    return original(name,*args,**kwargs)
builtins.__import__ = checked
import viewer_live, telemetry, runner_gait_ikfin
raise SystemExit(runner_gait_ikfin.main(['--dry-run','--telemetry','--telemetry-port','0','--vx','0','--duration','.1']))
'''
        result = subprocess.run([sys.executable, "-c", script], cwd=ROOT, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DRY RUN", result.stdout)


if __name__ == "__main__":
    unittest.main()
