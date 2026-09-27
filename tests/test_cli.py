from contextlib import redirect_stdout, redirect_stderr
import builtins
import io
from pathlib import Path
import signal
import subprocess
import sys
import unittest
from unittest.mock import patch

from keyboard_control import DeadmanKeys, Keyboard
from robot_config import SafetyError, load_config
import runner_gait_ikfin as runner


ROOT = Path(__file__).resolve().parents[1]


class KeyboardTests(unittest.TestCase):
    def test_no_keys_means_zero(self):
        self.assertEqual(DeadmanKeys(.65,.003,.015).command(0), (0.,0.,0.))

    def test_repeat_timeout_clears_motion_without_needing_keyup(self):
        keys = DeadmanKeys(.65,.003,.015)
        keys.feed("w", 10.)
        self.assertEqual(keys.command(10.1), (.003,0.,0.))
        self.assertEqual(keys.command(10.7), (0.,0.,0.))

    def test_opposites_cancel_and_q_is_yaw_not_quit(self):
        keys = DeadmanKeys(.65,.003,.015)
        for key in "wsadqe":
            keys.feed(key, 1.)
        self.assertEqual(keys.command(1.1), (0.,0.,0.))
        keys.feed(" ", 2.); keys.feed("q", 2.1)
        self.assertEqual(keys.command(2.2), (0.,0.,.015))

    def test_stop_and_pose_keys_clear_all_previous_motion(self):
        for stop in (" ","x","\x1b","1","2","3","4"):
            keys = DeadmanKeys(.65,.003,.015)
            keys.feed("w", 10.); keys.feed("q", 10.); keys.feed(stop, 10.1)
            self.assertEqual(keys.command(10.2), (0.,0.,0.))

    def test_ansi_sequences_do_not_leak_numbers_into_pose_commands(self):
        keys = Keyboard()
        keys.buffer = "\x1b[1;3A\x1b[12~\x1bOBx"
        self.assertEqual(keys.parse_buffer(1.), ["w", "s", "x"])

    def test_split_arrow_sequence_and_lone_escape(self):
        keys = Keyboard()
        keys.buffer = "\x1b[1;"
        self.assertEqual(keys.parse_buffer(1.), [])
        keys.buffer += "2D"
        self.assertEqual(keys.parse_buffer(1.01), ["a"])
        keys.buffer = "\x1b"
        self.assertEqual(keys.parse_buffer(2.), [])
        self.assertEqual(keys.parse_buffer(2.1), ["\x1b"])


class CLITests(unittest.TestCase):
    def invoke(self, args):
        out = io.StringIO()
        with redirect_stdout(out), redirect_stderr(out):
            code = runner.main(args)
        return code, out.getvalue()

    def test_check_is_offline_and_explains_physical_blockers(self):
        code, text = self.invoke(["--check"])
        self.assertEqual(code, 0)
        self.assertIn("Configuration + STAND IK: OK", text)
        self.assertIn("Hardware: BLOCKED", text)
        self.assertIn("LIE: DISABLED", text)

    def test_hardware_rejected_before_output_open(self):
        with patch.object(runner.HardwareOutput, "open", side_effect=AssertionError("must not open")):
            code, text = self.invoke(["--hardware", "--initial-pose", "STAND"])
        self.assertEqual(code, 2)
        self.assertIn("HARDWARE BLOCKED", text)

    def test_hardware_needs_explicit_initial_pose(self):
        code, text = self.invoke(["--hardware"])
        self.assertEqual(code, 2)
        self.assertIn("--initial-pose", text)

    def test_negative_duration_nan_and_velocity_fail(self):
        for args in (["--duration", "-5"], ["--duration", "nan"], ["--vx", "nan"], ["--wz", "inf"]):
            code, text = self.invoke(args)
            self.assertEqual(code, 2)
            self.assertIn("STOPPED", text)

    def test_import_and_dry_run_forbid_all_hardware_imports(self):
        script = '''
import builtins, signal
before = signal.getsignal(signal.SIGINT)
real_import = builtins.__import__
def checked(name, *args, **kwargs):
    if name.split('.')[0] in {'adafruit_servokit','smbus2','board','busio','adafruit_pca9685'}:
        raise AssertionError('hardware import attempted: '+name)
    return real_import(name,*args,**kwargs)
builtins.__import__ = checked
import runner_gait_ikfin
assert before == signal.getsignal(signal.SIGINT)
raise SystemExit(runner_gait_ikfin.main(['--dry-run','--vx','0','--duration','0.1']))
'''
        result = subprocess.run([sys.executable,"-c",script],cwd=ROOT,capture_output=True,text=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn("OFFLINE / NO I2C",result.stdout)

    def test_can_launch_from_unrelated_working_directory(self):
        result = subprocess.run([sys.executable,str(ROOT/"runner_gait_ikfin.py"),"--check"],
                                cwd=ROOT.parent,capture_output=True,text=True,timeout=20)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_runtime_interrupt_closes_output_without_scripted_recline(self):
        created = []
        class Output:
            cleanup_errors = []
            def __init__(self, config):
                self.closed = False; self.opened = False; self.writes = 0; created.append(self)
            def open(self, initial): self.opened = True
            def write(self, pose, dt): self.writes += 1
            def close(self): self.closed = True
        sigterm = signal.getsignal(signal.SIGTERM)
        with patch.object(runner,"DryRunOutput",Output), patch.object(runner.MotionController,"update",side_effect=KeyboardInterrupt):
            with redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
                runner.run_realtime(load_config(),(0,0,0),.1,"STAND")
        self.assertTrue(created[0].opened)
        self.assertTrue(created[0].closed)
        self.assertEqual(created[0].writes,0)
        self.assertEqual(signal.getsignal(signal.SIGTERM),sigterm)

    def test_noninteractive_keyboard_fails_before_hardware_open(self):
        class BadKeyboard:
            def __enter__(self): raise ValueError("not a tty")
            def __exit__(self,*args): pass
        with patch.object(runner,"Keyboard",BadKeyboard), patch.object(runner.HardwareOutput,"open",side_effect=AssertionError):
            with self.assertRaises(ValueError):
                runner.run_realtime(load_config(),(0,0,0),.1,"STAND",hardware=True,keyboard=True)


if __name__ == "__main__":
    unittest.main()
