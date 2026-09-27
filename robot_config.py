"""Validated robot configuration. Reading this module never opens hardware."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

LEGS = ("FL", "FR", "RL", "RR")
JOINTS = tuple(f"{leg}_{joint}" for leg in LEGS for joint in ("sh", "up", "lo"))
DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "robot.json"


class SafetyError(ValueError):
    """Reject a command before writing any of its channels."""


def finite_number(value, label, lo=None, hi=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise SafetyError(f"{label}: expected a finite number")
    if lo is not None and value < lo or hi is not None and value > hi:
        raise SafetyError(f"{label}: {value} outside [{lo}, {hi}]")
    return float(value)


def integer(value, label, lo, hi):
    finite_number(value, label, lo, hi)
    if not isinstance(value, int):
        raise SafetyError(f"{label}: expected an integer")
    return value


class RobotConfig:
    def __init__(self, data, poses, source=None):
        try:
            if data["schema_version"] != 1:
                raise SafetyError("Unsupported robot configuration schema")
            self.source = str(source) if source else "in-memory"
            self.data, self.poses = data, poses
            self.geometry = data["geometry"]
            self.gait = data["gait"]
            self.control = data["control"]
            self.hardware = data["hardware"]
            self.servos = data["servos"]
            if set(self.servos) != set(JOINTS):
                raise SafetyError("Exactly the 12 named servos are required; no inferred 90-degree values")
            for k in ("body_length_m", "body_width_m", "hip_link_m", "upper_link_m", "lower_link_m"):
                finite_number(self.geometry[k], f"geometry.{k}", 0.001, 1.0)
            if type(self.geometry["verified_on_robot"]) is not bool:
                raise SafetyError("geometry.verified_on_robot must be a boolean")
            length, width = self.geometry["body_length_m"], self.geometry["body_width_m"]
            self.hips = {leg: np.array([(1 if leg[0] == "F" else -1) * length / 2,
                                        (1 if leg[1] == "L" else -1) * width / 2, 0.]) for leg in LEGS}
            if set(data["nominal_feet_body_m"]) != set(LEGS):
                raise SafetyError("All four nominal foot positions are required")
            self.nominal = {}
            for leg in LEGS:
                point = data["nominal_feet_body_m"][leg]
                if len(point) != 3:
                    raise SafetyError(f"{leg}: foot target needs x,y,z")
                self.nominal[leg] = np.array([finite_number(v, f"{leg} coordinate", -1., 1.) for v in point])
                if self.nominal[leg][2] >= -0.01:
                    raise SafetyError(f"{leg}: nominal feet must be below the body")
            if max(p[2] for p in self.nominal.values()) - min(p[2] for p in self.nominal.values()) > 1e-8:
                raise SafetyError("This flat-ground crawl requires equal nominal foot heights")
            seen = set()
            for name, s in self.servos.items():
                ch = integer(s["channel"], f"{name}.channel", 0, 15)
                if ch in seen:
                    raise SafetyError(f"Duplicate PCA9685 channel: {ch}")
                seen.add(ch)
                if isinstance(s["direction"], bool) or s["direction"] not in (-1, 1):
                    raise SafetyError(f"{name}: direction must be -1 or +1")
                finite_number(s["offset_deg"], f"{name}.offset", -180, 180)
                lo = finite_number(s["min_deg"], f"{name}.min", 0, 180)
                hi = finite_number(s["max_deg"], f"{name}.max", 0, 180)
                if lo >= hi:
                    raise SafetyError(f"{name}: min must be less than max")
                finite_number(s["max_rate_deg_s"], f"{name}.max_rate", 1, 180)
                pmin = integer(s["min_pulse_us"], f"{name}.min_pulse", 400, 2600)
                pmax = integer(s["max_pulse_us"], f"{name}.max_pulse", 400, 2600)
                if pmin >= pmax:
                    raise SafetyError(f"{name}: pulse interval is reversed")
                actuation = finite_number(s["actuation_range_deg"], f"{name}.actuation", 1, 180)
                if hi > actuation:
                    raise SafetyError(f"{name}: angle limit exceeds configured actuation range")
            if len(self.gait["order"]) != 4 or set(self.gait["order"]) != set(LEGS):
                raise SafetyError("Crawl order must name each leg exactly once")
            for key, lo, hi in (("transfer_time_s", .2, 5), ("swing_time_s", .2, 3),
                                ("lift_m", .001, .04), ("max_stride_m", .001, .06),
                                ("max_translation_mps", .001, .05), ("max_yaw_rps", .001, .3),
                                ("translation_accel_mps2", .001, .2), ("yaw_accel_rps2", .001, 1),
                                ("support_margin_m", .001, .04), ("max_body_shift_m", .01, .15),
                                ("settle_tolerance_m", 1e-6, .002)):
                finite_number(self.gait[key], f"gait.{key}", lo, hi)
            if len(self.gait["com_offset_xy_m"]) != 2:
                raise SafetyError("COM offset must contain x,y")
            for value in self.gait["com_offset_xy_m"]:
                finite_number(value, "COM offset", -.10, .10)
            for key, lo, hi in (("rate_hz", 20, 100), ("max_loop_gap_s", .05, .5),
                                ("keyboard_timeout_s", .1, 1), ("keyboard_speed_mps", .001, .02),
                                ("keyboard_yaw_rps", .001, .1), ("pose_transition_s", .5, 10),
                                ("max_tracking_error_deg", 1, 15)):
                finite_number(self.control[key], f"control.{key}", lo, hi)
            if self.control["max_loop_gap_s"] <= 1 / self.control["rate_hz"]:
                raise SafetyError("Loop gap threshold must exceed one control period")
            integer(self.hardware["pca9685_address"], "PCA address", 3, 119)
            integer(self.hardware["pwm_frequency_hz"], "PWM frequency", 40, 60)
            if type(self.hardware["calibration_verified_on_robot"]) is not bool:
                raise SafetyError("hardware.calibration_verified_on_robot must be a boolean")
            relay = self.hardware["relay"]
            if type(relay["enabled"]) is not bool:
                raise SafetyError("relay.enabled must be a boolean")
            integer(relay["bus"], "relay.bus", 0, 20)
            integer(relay["address"], "relay.address", 3, 119)
            integer(relay["register"], "relay.register", 0, 255)
            if relay["address"] == self.hardware["pca9685_address"]:
                raise SafetyError("Relay and PCA9685 addresses must be different")
            for key in ("on_data", "off_data"):
                values = relay[key]
                if not isinstance(values, list) or not 1 <= len(values) <= 32:
                    raise SafetyError(f"relay.{key}: expected 1..32 bytes")
                for value in values:
                    integer(value, f"relay.{key}", 0, 255)
            if relay["on_data"] == relay["off_data"]:
                raise SafetyError("Relay ON and OFF messages must differ")
            self.pose("STAND")
        except (KeyError, TypeError, OverflowError) as exc:
            raise SafetyError(f"Incomplete or malformed configuration: {exc}") from exc

    def validate_pose(self, pose):
        if set(pose) != set(JOINTS):
            raise SafetyError("Servo frame must contain exactly all 12 joints")
        out = {}
        for name in JOINTS:
            s = self.servos[name]
            out[name] = finite_number(pose[name], name, s["min_deg"], s["max_deg"])
        return out

    def pose(self, name):
        if name not in self.poses:
            raise SafetyError(f"Unknown pose {name!r}; available: {', '.join(self.poses)}")
        raw = self.poses[name]
        if set(raw) != set(JOINTS):
            raise SafetyError(f"Pose {name} is missing joints or has unknown joints")
        return self.validate_pose({joint: finite_number(raw[joint], f"{name}.{joint}", 0, 180)
                                  + self.servos[joint]["offset_deg"] for joint in JOINTS})

    def hardware_blockers(self):
        problems = []
        if not self.geometry["verified_on_robot"]:
            problems.append("geometry.verified_on_robot=false: measure the links and neutral foot positions")
        if not self.hardware["calibration_verified_on_robot"]:
            problems.append("hardware.calibration_verified_on_robot=false: verify channels, directions, "
                            "pulse ranges, joint limits, STAND and relay behavior on the supported robot")
        if not self.hardware["relay"]["enabled"]:
            problems.append("A verified working power relay is required for this hardware runner")
        return problems


def load_config(path=DEFAULT_CONFIG, poses_path=None):
    path = Path(path).expanduser().resolve()
    poses_path = Path(poses_path).expanduser().resolve() if poses_path else path.with_name("poses.json")
    try:
        with path.open(encoding="utf-8") as f:
            data = json.load(f)
        with poses_path.open(encoding="utf-8") as f:
            poses = json.load(f)
        return RobotConfig(data, poses, path)
    except (OSError, json.JSONDecodeError) as exc:
        raise SafetyError(f"Cannot load robot configuration: {exc}") from exc
