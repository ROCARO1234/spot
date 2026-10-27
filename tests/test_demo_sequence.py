import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from demo_sequence import DemoSequence
from pose_validation import fingerprint, require_validation, main as record_validation
from probe_pose import probe_config, run_probe
from robot_config import SafetyError, load_config
from robot_io import DryRunOutput
import runner_gait_ikfin as runner


class DemoTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()

    def test_rise_walk_midair_stop_lower_and_finish(self):
        seq = DemoSequence(self.config)
        phases = [seq.phase]
        output = DryRunOutput(self.config)
        output.open(seq.last_pose)
        active = 0
        for _ in range(4000):
            if seq.phase == 'ACTIVE':
                active += 1
                seq.set_command(.003, 0, 0)
                if seq.frame.swing_legs:
                    seq.request_stop()
            output.write(seq.update(.02), .02)
            if seq.phase != phases[-1]:
                phases.append(seq.phase)
            if seq.phase == 'LOWERING':
                self.assertEqual(seq.frame.swing_legs, ())
            if seq.settled:
                break
        self.assertGreater(active, 1)
        self.assertEqual(phases, ['RISING', 'ACTIVE', 'STOPPING', 'LOWERING', 'COMPLETE'])
        self.assertEqual(seq.last_pose, self.config.pose('CROUCH_READY'))

    def test_stop_during_rise_and_no_queued_motion(self):
        seq = DemoSequence(self.config)
        seq.set_command(.003, 0, 0)
        seq.request_stop()
        for _ in range(1000):
            seq.update(.02)
            if seq.settled:
                break
        self.assertTrue(seq.settled)
        self.assertEqual(seq.planner.lift_count, 0)

    def test_missing_record_blocks_before_hardware_open(self):
        with patch.object(runner.HardwareOutput, 'open', side_effect=AssertionError('must not arm')):
            with self.assertRaisesRegex(SafetyError, 'validation record'):
                runner.run_realtime(self.config, (0,0,0), 1, 'CROUCH_READY',
                                    hardware=True, demo=True)

    def test_record_is_bound_to_calibration_and_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'validation.json'
            record_validation(['record', '--config', self.config.source, '--pose', 'CROUCH_READY',
                               '--operator', 'unit-test', '--evidence', 'SYNTHETIC TEST ONLY',
                               '--confirmed-supported-roundtrip', '--output', str(path)])
            require_validation(self.config, 'CROUCH_READY', path)
            modified = load_config()
            modified.servos['FL_sh']['offset_deg'] += .1
            with self.assertRaises(SafetyError):
                require_validation(modified, 'CROUCH_READY', path)
            record = json.loads(path.read_text())
            record['supported_roundtrip_confirmed'] = False
            path.write_text(json.dumps(record))
            with self.assertRaises(SafetyError):
                require_validation(self.config, 'CROUCH_READY', path)

    def test_probe_does_not_change_calibration_and_extremes_stay_disabled(self):
        before = copy.deepcopy(self.config.data)
        run_probe(probe_config(self.config, 'CROUCH_READY', .1))
        self.assertEqual(self.config.data, before)
        for name in ('LIE', 'UP_READY'):
            with self.assertRaises(SafetyError):
                probe_config(self.config, name, .1)

    def test_demo_emergency_skips_lowering_and_closes_output(self):
        with patch.object(runner.DryRunOutput, 'close') as close, \
             patch.object(DemoSequence, 'update', side_effect=KeyboardInterrupt), \
             patch.object(DemoSequence, 'request_stop', side_effect=AssertionError('no normal lowering')):
            with self.assertRaises(KeyboardInterrupt):
                runner.run_realtime(self.config, (0,0,0), 1, 'CROUCH_READY', demo=True)
            close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
