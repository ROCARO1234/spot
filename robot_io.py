"""Fail-closed outputs. Hardware libraries are imported only by an explicit arm.

The relay messages come from the user's November 2025 i2c_leg.py. Electrical
behavior cannot be verified here. No automatic fallback to simulation follows
a hardware error: the program reports the error and attempts power-off.
"""
from __future__ import annotations

from robot_config import JOINTS, SafetyError, finite_number


def checked_frame(config, pose, previous=None, dt=None):
    result = config.validate_pose(pose)
    if previous is not None:
        finite_number(dt, "output dt", 1e-6, config.control["max_loop_gap_s"])
        for name in JOINTS:
            allowed = config.servos[name]["max_rate_deg_s"] * dt + 1e-7
            if abs(result[name] - previous[name]) > allowed:
                raise SafetyError(f"{name}: output exceeds configured slew rate")
    return result


class DryRunOutput:
    def __init__(self, config):
        self.config = config
        self.last_pose = None
        self.frames = 0
        self.armed = False
        self.cleanup_errors = []

    def open(self, initial_pose):
        self.last_pose = checked_frame(self.config, initial_pose)
        self.armed = True

    def write(self, pose, dt):
        if not self.armed:
            raise SafetyError("Dry-run output is not open")
        self.last_pose = checked_frame(self.config, pose, self.last_pose, dt)
        self.frames += 1

    def close(self):
        self.armed = False


class HardwareOutput:
    def __init__(self, config, *, kit_factory=None, bus_factory=None):
        self.config = config
        self.kit_factory, self.bus_factory = kit_factory, bus_factory
        self.kit = self.bus = None
        self.last_pose = None
        self.armed = False
        self.closed = False
        self.cleanup_errors = []

    def _relay(self, on):
        relay = self.config.hardware["relay"]
        if self.bus is None:
            raise SafetyError("Relay bus is not available")
        self.bus.write_i2c_block_data(relay["address"], relay["register"],
                                     relay["on_data"] if on else relay["off_data"])

    def open(self, initial_pose):
        if self.closed or self.armed:
            raise SafetyError("Output cannot be re-armed in the same process")
        frame = checked_frame(self.config, initial_pose)
        blockers = self.config.hardware_blockers()
        if blockers:
            raise SafetyError("HARDWARE BLOCKED:\n  " + "\n  ".join(blockers))
        # Import failure happens before opening either bus or enabling power.
        if self.kit_factory is None:
            try:
                from adafruit_servokit import ServoKit
            except (ImportError, RuntimeError) as exc:
                # Blinka wraps a missing platform library in RuntimeError.
                raise SafetyError(
                    f"ServoKit/Blinka import failed: {exc}. "
                    "See docs/usage.md: on Raspberry Pi, verify python3-lgpio "
                    "with /usr/bin/python3 and use a --system-site-packages venv; "
                    "install requirements-hardware.txt in that same environment. "
                    "No I2C bus was opened by this output."
                ) from exc
            self.kit_factory = ServoKit
        if self.bus_factory is None:
            from smbus2 import SMBus
            self.bus_factory = SMBus
        try:
            self.bus = self.bus_factory(self.config.hardware["relay"]["bus"])
            self._relay(False)
            self.kit = self.kit_factory(channels=16, address=self.config.hardware["pca9685_address"],
                                        frequency=self.config.hardware["pwm_frequency_hz"])
            for name in JOINTS:
                entry = self.config.servos[name]
                servo = self.kit.servo[entry["channel"]]
                servo.set_pulse_width_range(entry["min_pulse_us"], entry["max_pulse_us"])
                servo.actuation_range = entry["actuation_range_deg"]
                servo.angle = None
            # Stage a COMPLETE validated pose while motor power remains OFF.
            # The first powered pose cannot be interpolated from an unknown
            # physical position: the operator must support and align the robot.
            for name in JOINTS:
                self.kit.servo[self.config.servos[name]["channel"]].angle = frame[name]
            self.last_pose = frame
            self._relay(True)
            self.armed = True
        except BaseException:
            self.close()
            raise

    def write(self, pose, dt):
        if not self.armed or self.closed:
            raise SafetyError("Hardware output is disarmed")
        try:
            frame = checked_frame(self.config, pose, self.last_pose, dt)
            for name in JOINTS:
                self.kit.servo[self.config.servos[name]["channel"]].angle = frame[name]
            self.last_pose = frame
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.armed = False
        # Attempt power-off FIRST, even if an individual PWM channel failed.
        if self.bus is not None:
            try:
                self._relay(False)
            except Exception as exc:
                self.cleanup_errors.append(f"Relay power-off failed: {exc}")
        if self.kit is not None:
            for name in JOINTS:
                try:
                    self.kit.servo[self.config.servos[name]["channel"]].angle = None
                except Exception as exc:
                    self.cleanup_errors.append(f"PWM disable {name} failed: {exc}")
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception as exc:
                self.cleanup_errors.append(f"Bus close failed: {exc}")
        self.bus = None
        self.kit = None
