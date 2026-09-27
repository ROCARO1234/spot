import copy
import unittest

from robot_config import JOINTS, RobotConfig, SafetyError, load_config
from robot_io import DryRunOutput, HardwareOutput


class FakeServo:
    def __init__(self, channel, operations):
        self.channel, self.operations = channel, operations
        self.fail_value = False
        self.fail_disable = False
        self._angle = None

    def set_pulse_width_range(self, lo, hi):
        self.operations.append(("pulse", self.channel, lo, hi))

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, value):
        if value is None and self.fail_disable or value is not None and self.fail_value:
            self.operations.append(("pwm_failure", self.channel, value))
            raise OSError("injected PWM failure")
        self.operations.append(("angle", self.channel, value))
        self._angle = value


class FakeKit:
    def __init__(self, operations, **kwargs):
        self.operations = operations
        self.operations.append(("kit_open", kwargs))
        self.servo = [FakeServo(i, operations) for i in range(16)]


class FakeBus:
    def __init__(self, operations, bus):
        self.operations = operations
        self.operations.append(("bus_open", bus))
        self.fail_on = False
        self.fail_off = False

    def write_i2c_block_data(self, addr, reg, data):
        self.operations.append(("relay", addr, reg, data.copy()))
        if self.fail_on and data == [2] or self.fail_off and data == [1]:
            raise OSError("injected relay failure")

    def close(self):
        self.operations.append(("bus_close",))


class SafetyIOTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config()
        data = copy.deepcopy(cfg.data)
        # Review flags are enabled ONLY in this fake, in-memory fixture.
        data["geometry"]["verified_on_robot"] = True
        data["hardware"]["calibration_verified_on_robot"] = True
        self.config = RobotConfig(data, copy.deepcopy(cfg.poses))
        self.operations = []
        self.kit = None
        self.bus = None

    def kit_factory(self, **kwargs):
        self.kit = FakeKit(self.operations, **kwargs)
        return self.kit

    def bus_factory(self, number):
        self.bus = FakeBus(self.operations, number)
        return self.bus

    def make_output(self, config=None, kit_factory=None, bus_factory=None):
        return HardwareOutput(config or self.config, kit_factory=kit_factory or self.kit_factory,
                              bus_factory=bus_factory or self.bus_factory)

    def test_unverified_config_does_not_call_any_hardware_factory(self):
        def forbidden(*args, **kwargs):
            self.fail("Hardware factory called for an unverified robot")
        output = HardwareOutput(load_config(), kit_factory=forbidden, bus_factory=forbidden)
        with self.assertRaisesRegex(SafetyError, "HARDWARE BLOCKED"):
            output.open(self.config.pose("STAND"))
        output.close()

    def test_import_and_construction_do_not_touch_hardware(self):
        self.make_output().close()
        self.assertEqual(self.operations, [])

    def test_full_frame_staged_with_relay_off_before_on(self):
        output = self.make_output()
        output.open(self.config.pose("STAND"))
        on = next(i for i, op in enumerate(self.operations) if op[0] == "relay" and op[3] == [2])
        off = next(i for i, op in enumerate(self.operations) if op[0] == "relay" and op[3] == [1])
        commands = [(i, op) for i, op in enumerate(self.operations) if op[0] == "angle" and op[2] is not None]
        self.assertEqual(len(commands), 12)
        self.assertTrue(all(off < i < on for i, _ in commands))
        self.assertEqual({op[1] for _, op in commands}, {s["channel"] for s in self.config.servos.values()})
        self.assertTrue(output.armed)
        output.close()

    def test_relay_protocol_and_existing_servokit_pulse_range_preserved(self):
        output = self.make_output(); output.open(self.config.pose("STAND"))
        relays = [op for op in self.operations if op[0] == "relay"]
        self.assertEqual(relays[:2], [("relay", 8, 1, [1]), ("relay", 8, 1, [2])])
        pulses = [op for op in self.operations if op[0] == "pulse"]
        self.assertTrue(all(op[2:] == (750,2250) for op in pulses))
        kit = next(op[1] for op in self.operations if op[0] == "kit_open")
        self.assertEqual(kit, {"channels":16, "address":64, "frequency":50})
        output.close()

    def test_initial_invalid_pose_does_not_open_i2c(self):
        output = self.make_output()
        bad = self.config.pose("STAND"); bad["RR_lo"] = 181
        with self.assertRaises(SafetyError):
            output.open(bad)
        self.assertEqual(self.operations, [])

    def test_invalid_last_channel_prevents_all_frame_writes_and_cuts_power(self):
        output = self.make_output(); output.open(self.config.pose("STAND"))
        self.operations.clear()
        bad = self.config.pose("STAND"); bad["RR_lo"] = 500
        with self.assertRaises(SafetyError):
            output.write(bad, .02)
        self.assertFalse(any(op[0] == "angle" and op[2] is not None for op in self.operations))
        self.assertEqual(self.operations[0], ("relay",8,1,[1]))
        self.assertFalse(output.armed)

    def test_velocity_limit_violation_cuts_power_before_any_new_angles(self):
        output = self.make_output(); output.open(self.config.pose("STAND"))
        self.operations.clear()
        bad = self.config.pose("STAND"); bad["FL_up"] += 10
        with self.assertRaises(SafetyError):
            output.write(bad, .02)
        self.assertEqual(self.operations[0], ("relay",8,1,[1]))
        self.assertFalse(any(op[0] == "angle" and op[2] is not None for op in self.operations))

    def test_loop_gap_exceeded_cuts_power(self):
        output = self.make_output(); output.open(self.config.pose("STAND"))
        with self.assertRaises(SafetyError):
            output.write(self.config.pose("STAND"), 1.)
        self.assertFalse(output.armed)
        self.assertIn(("relay",8,1,[1]), self.operations)

    def test_pwm_failure_attempts_relay_off_even_if_disable_also_fails(self):
        output = self.make_output(); output.open(self.config.pose("STAND"))
        channel = self.config.servos["FR_up"]["channel"]
        self.kit.servo[channel].fail_value = True
        self.kit.servo[channel].fail_disable = True
        self.operations.clear()
        with self.assertRaises(OSError):
            output.write(self.config.pose("STAND"), .02)
        failure = next(i for i, op in enumerate(self.operations) if op[0] == "pwm_failure")
        self.assertEqual(self.operations[failure+1], ("relay",8,1,[1]))
        self.assertEqual(self.operations[-1], ("bus_close",))
        self.assertTrue(output.cleanup_errors)

    def test_driver_initialization_failure_never_powers_on(self):
        def bad_factory(**kwargs):
            raise OSError("driver initialization failed")
        output = self.make_output(kit_factory=bad_factory)
        with self.assertRaises(OSError):
            output.open(self.config.pose("STAND"))
        self.assertNotIn(("relay",8,1,[2]), self.operations)
        self.assertEqual(self.operations[-1], ("bus_close",))

    def test_relay_on_failure_rolls_back(self):
        def bus_factory(number):
            bus = self.bus_factory(number); bus.fail_on = True
            return bus
        output = self.make_output(bus_factory=bus_factory)
        with self.assertRaises(OSError):
            output.open(self.config.pose("STAND"))
        self.assertEqual([op[3] for op in self.operations if op[0] == "relay"], [[1],[2],[1]])
        self.assertFalse(output.armed)

    def test_shutdown_error_is_reported_and_remaining_shutdown_attempted(self):
        output = self.make_output(); output.open(self.config.pose("STAND"))
        self.bus.fail_off = True
        output.close()
        self.assertTrue(any("Relay power-off failed" in msg for msg in output.cleanup_errors))
        self.assertEqual(self.operations[-1], ("bus_close",))
        self.assertEqual(sum(op[0] == "angle" and op[2] is None for op in self.operations), 24)

    def test_close_is_idempotent_and_disarmed_object_cannot_restart(self):
        output = self.make_output(); output.open(self.config.pose("STAND")); output.close()
        count = len(self.operations)
        output.close()
        self.assertEqual(len(self.operations), count)
        with self.assertRaises(SafetyError):
            output.open(self.config.pose("STAND"))

    def test_dry_run_validates_without_requiring_review_flags(self):
        output = DryRunOutput(load_config()); output.open(self.config.pose("STAND"))
        output.write(self.config.pose("STAND"), .02)
        self.assertEqual(output.frames, 1)
        output.close()


if __name__ == "__main__":
    unittest.main()
