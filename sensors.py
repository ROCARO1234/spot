"""Optional measurements, with no hardware imports or stabilization side effects.

A backend must return measurements timestamped with this process' monotonic
clock. Commanded angles and simulated values must never be supplied as feedback.
Device drivers and electrical conversion are intentionally not implemented yet.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Protocol


JOINT_NAMES = tuple(f"{leg}_{part}" for leg in ("FL", "FR", "RL", "RR")
                    for part in ("sh", "up", "lo"))


class SensorBackend(Protocol):
    def read(self) -> dict | None:
        """Return a measured sample in the documented schema, or None."""


@dataclass(frozen=True)
class SensorReading:
    available: bool
    reason: str
    source: str | None = None
    timestamp_monotonic_s: float | None = None
    age_s: float | None = None
    imu: dict | None = None
    joint_angles_deg: dict | None = None


def _number(value, name, lo=None, hi=None):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"{name}: expected finite number")
    if lo is not None and value < lo or hi is not None and value > hi:
        raise ValueError(f"{name}: outside allowed range")
    return float(value)


def load_sensor_settings(path=None):
    """No file means disabled; never probe a bus to discover missing devices."""
    if path is None:
        return {"schema_version": 1, "enabled": False, "max_age_s": .2, "backend": None}
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return json.load(stream)


class SensorHub:
    def __init__(self, settings=None, *, backend: SensorBackend | None = None,
                 clock=time.monotonic):
        self.settings = dict(load_sensor_settings() if settings is None else settings)
        if set(self.settings) != {"schema_version", "enabled", "max_age_s", "backend"}:
            raise ValueError("Sensor settings require schema_version, enabled, max_age_s and backend")
        if type(self.settings["schema_version"]) is not int or self.settings["schema_version"] != 1:
            raise ValueError("Unsupported sensor schema")
        if type(self.settings["enabled"]) is not bool:
            raise ValueError("Sensor enabled must be boolean")
        self.max_age_s = _number(self.settings["max_age_s"], "max_age_s", .001, 10)
        if self.settings["backend"] not in (None, "injected"):
            raise ValueError("No hardware sensor driver is implemented; only an injected backend is supported")
        if backend is not None and self.settings["backend"] != "injected":
            raise ValueError("Explicit backend='injected' is required for a supplied backend")
        self.backend, self.clock = backend, clock
        self.last_timestamp = None

    def poll(self):
        if not self.settings["enabled"]:
            return SensorReading(False, "disabled; sensors not mounted")
        if self.backend is None:
            return SensorReading(False, "no measurement backend; hardware models and wiring unresolved")
        try:
            sample = self.backend.read()
            if sample is None:
                return SensorReading(False, "backend has no measurement")
            if not isinstance(sample, dict) or set(sample) != {
                    "schema_version", "source", "timestamp_monotonic_s", "imu", "joint_angles_deg"}:
                raise ValueError("measurement schema fields do not match")
            if type(sample["schema_version"]) is not int or sample["schema_version"] != 1:
                raise ValueError("unsupported measurement schema")
            source = sample["source"]
            if not isinstance(source, str) or not source.strip() or len(source) > 200:
                raise ValueError("measurement source must identify the backend")
            timestamp = _number(sample["timestamp_monotonic_s"], "measurement timestamp", 0)
            now = _number(self.clock(), "monotonic clock", 0)
            if timestamp > now:
                raise ValueError("measurement timestamp is in the future")
            if self.last_timestamp is not None and timestamp < self.last_timestamp:
                raise ValueError("measurement timestamp went backwards")
            age = now - timestamp
            if age > self.max_age_s:
                return SensorReading(False, "stale measurement", source, timestamp, age)
            imu = sample["imu"]
            if imu is not None:
                if not isinstance(imu, dict) or set(imu) != {"accel_mps2", "gyro_rps"}:
                    raise ValueError("IMU requires accel_mps2 and gyro_rps")
                checked_imu = {}
                for name in ("accel_mps2", "gyro_rps"):
                    vector = imu[name]
                    if not isinstance(vector, (list, tuple)) or len(vector) != 3:
                        raise ValueError(f"{name}: requires three body-frame components")
                    checked_imu[name] = tuple(_number(v, name) for v in vector)
                imu = checked_imu
            joints = sample["joint_angles_deg"]
            if joints is not None:
                if not isinstance(joints, dict) or set(joints) != set(JOINT_NAMES):
                    raise ValueError("Feedback requires exactly the 12 calibrated joint angles")
                joints = {name: _number(joints[name], name, 0, 180) for name in JOINT_NAMES}
            if imu is None and joints is None:
                return SensorReading(False, "sample contains no measurements", source, timestamp, age)
            self.last_timestamp = timestamp
            return SensorReading(True, "measured; no stabilization enabled", source, timestamp, age, imu, joints)
        except Exception as exc:
            # Drop the entire invalid frame; never substitute the last angles,
            # commanded positions, zero vectors, or a generated IMU attitude.
            return SensorReading(False, f"measurement rejected: {type(exc).__name__}: {exc}")
