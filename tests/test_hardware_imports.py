import builtins
import copy
from contextlib import redirect_stderr, redirect_stdout
import io
import unittest
from unittest.mock import Mock, patch

from robot_config import RobotConfig, SafetyError, load_config
from robot_io import HardwareOutput
import runner_gait_ikfin as runner


class HardwareImportTests(unittest.TestCase):
    def setUp(self):
        config = load_config()
        data = copy.deepcopy(config.data)
        # In-memory fixture only. No file or hardware state is changed.
        data['geometry']['verified_on_robot'] = True
        data['hardware']['calibration_verified_on_robot'] = True
        self.config = RobotConfig(data, copy.deepcopy(config.poses))

    def broken_import(self, error):
        real_import = builtins.__import__
        def injected(name, *args, **kwargs):
            if name == 'adafruit_servokit':
                raise error
            return real_import(name, *args, **kwargs)
        return injected

    def lgpio_error(self):
        try:
            raise ModuleNotFoundError("No module named 'lgpio'", name='lgpio')
        except ModuleNotFoundError as cause:
            error = RuntimeError("The platform library 'lgpio' was not found")
            error.__cause__ = cause
            return error

    def test_blinka_runtime_error_is_actionable_before_any_bus_open(self):
        bus = Mock(side_effect=AssertionError('I2C must remain closed'))
        output = HardwareOutput(self.config, bus_factory=bus)
        error = self.lgpio_error()
        with patch('builtins.__import__', self.broken_import(error)):
            with self.assertRaisesRegex(SafetyError, 'system-site-packages') as caught:
                output.open(self.config.pose('STAND'))
        self.assertIs(caught.exception.__cause__, error)
        bus.assert_not_called()
        self.assertIsNone(output.kit)
        self.assertFalse(output.armed)
        output.close()
        self.assertEqual(output.cleanup_errors, [])

    def test_missing_servokit_preserves_original_import_cause(self):
        error = ModuleNotFoundError("No module named 'adafruit_servokit'", name='adafruit_servokit')
        bus = Mock(side_effect=AssertionError('I2C must remain closed'))
        output = HardwareOutput(self.config, bus_factory=bus)
        with patch('builtins.__import__', self.broken_import(error)):
            with self.assertRaisesRegex(SafetyError, 'adafruit_servokit') as caught:
                output.open(self.config.pose('STAND'))
        self.assertIs(caught.exception.__cause__, error)
        bus.assert_not_called()

    def test_cli_returns_two_for_blinka_failure_and_cleans_up(self):
        log = io.StringIO()
        error = self.lgpio_error()
        # Skip only trajectory computation; this test targets startup failure.
        with patch.object(runner, 'load_config', return_value=self.config), \
             patch.object(runner, 'simulate'), \
             patch('builtins.__import__', self.broken_import(error)), \
             redirect_stdout(log), redirect_stderr(log):
            code = runner.main(['--hardware', '--initial-pose', 'STAND', '--vx', '0', '--duration', '.1'])
        self.assertEqual(code, 2)
        self.assertIn('STOPPED:', log.getvalue())
        self.assertIn('lgpio', log.getvalue())
        self.assertNotIn('HARDWARE ARMED', log.getvalue())


if __name__ == '__main__':
    unittest.main()
