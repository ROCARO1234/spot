import copy
import unittest
from unittest.mock import patch

import numpy as np

import rear_setback
from robot_config import JOINTS, SafetyError, load_config
from robot_io import checked_frame


class RearSetbackTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()

    def test_rear_repositioning_preserves_other_feet_and_calibration(self):
        before = copy.deepcopy(self.config.data)
        plan = rear_setback.compile_probe(self.config, 25, stage='RL', fraction=1.)
        start = plan['frames'][0]['model_feet_body_m']
        outbound = [r for r in plan['frames'] if r['phase']=='SETBACK_RL']
        last = outbound[-1]['model_feet_body_m']
        self.assertAlmostEqual(last['RL'][0], start['RL'][0]-.025)
        self.assertAlmostEqual(last['RL'][2], start['RL'][2])
        self.assertAlmostEqual(max(r['model_feet_body_m']['RL'][2] for r in outbound),
                               start['RL'][2]+self.config.gait['lift_m'], delta=2e-5)
        for r in plan['frames']:
            for leg in ('FL','FR','RR'):
                np.testing.assert_allclose(r['model_feet_body_m'][leg],start[leg],atol=1e-12)
        self.assertEqual(self.config.data,before)
        self.assertFalse(plan['ground_validated'])

    def test_candidate_routes_obey_original_limits_and_reverse_to_low(self):
        for setback in (20,25,30):
            plan=rear_setback.compile_probe(self.config,setback,stage='rise',fraction=1.)
            previous=plan['frames'][0]['pose']
            for r in plan['frames'][1:]:
                checked_frame(self.config,r['pose'],previous,1/self.config.control['rate_hz'])
                previous=r['pose']
            for name in ('RISE','RETURN_RISE'):
                self.assertTrue(any(r['phase']==name for r in plan['frames']))
            stand=[r for r in plan['frames'] if r['phase']=='RISE'][-1]['pose']
            for j in JOINTS:
                self.assertAlmostEqual(stand[j],self.config.pose('STAND')[j])
                self.assertAlmostEqual(previous[j],self.config.pose('CROUCH_READY')[j])

    def test_fraction_scales_setback_and_clearance(self):
        plan=rear_setback.compile_probe(self.config,20,stage='RR',fraction=.25)
        start=plan['frames'][0]['model_feet_body_m']['RR']
        end=[r for r in plan['frames'] if r['phase']=='SETBACK_RR'][-1]['model_feet_body_m']['RR']
        self.assertAlmostEqual(end[0]-start[0],-.005)
        self.assertEqual(plan['lift_mm'],3.)

    def test_invalid_inputs_and_mismatched_plan_refused(self):
        for value in (0,31,True,float('nan')):
            with self.assertRaises(SafetyError):
                rear_setback.compile_probe(self.config,value)
        plan=rear_setback.compile_probe(self.config,20)
        plan['config_fingerprint']='incorrect'
        with patch.object(rear_setback.HardwareOutput,'open',side_effect=AssertionError('must not arm')):
            with self.assertRaises(SafetyError):
                rear_setback.run_probe(self.config,plan,hardware=True)

    def test_emergency_closes_output_without_scripted_return(self):
        plan=rear_setback.compile_probe(self.config,20)
        with patch.object(rear_setback.DryRunOutput,'write',side_effect=KeyboardInterrupt), \
             patch.object(rear_setback.DryRunOutput,'close') as close:
            with self.assertRaises(KeyboardInterrupt):
                rear_setback.run_probe(self.config,plan)
            close.assert_called_once()

    def test_corrupt_future_frame_is_rejected_before_power(self):
        plan=rear_setback.compile_probe(self.config,20)
        plan['frames'][20]['pose']['RL_up']=180.
        with patch.object(rear_setback.HardwareOutput,'open',side_effect=AssertionError('must not arm')):
            with self.assertRaises(SafetyError):
                rear_setback.run_probe(self.config,plan,hardware=True)

    def test_cli_requires_explicit_support_log_and_validation_before_power(self):
        base=['--config',self.config.source,'--rear-setback-mm','20','--hardware']
        with patch.object(rear_setback.HardwareOutput,'open',side_effect=AssertionError('must not arm')):
            self.assertEqual(rear_setback.main(base),2)
            self.assertEqual(rear_setback.main(base+['--confirm-supported','--log','unused-test-log.jsonl']),2)

    def test_observation_hold_is_stationary_and_does_not_enlarge_motion(self):
        original=rear_setback.compile_probe(self.config,20,stage='RL',fraction=.5)
        plan=rear_setback.compile_probe(self.config,20,stage='RL',fraction=.5,hold_s=3.)
        endpoint=[r for r in plan['frames'] if r['phase']=='SETBACK_RL'][-1]
        held=[r for r in plan['frames'] if r['phase']=='HOLD_SETBACK_RL']
        self.assertEqual(len(held),150)
        for frame in held:
            self.assertEqual(frame['pose'],endpoint['pose'])
            self.assertEqual(frame['model_feet_body_m'],endpoint['model_feet_body_m'])
        self.assertAlmostEqual(plan['duration_s']-original['duration_s'],3.)
        for j in JOINTS:
            self.assertAlmostEqual(max(r['pose'][j] for r in plan['frames']),
                                   max(r['pose'][j] for r in original['frames']))
            self.assertAlmostEqual(min(r['pose'][j] for r in plan['frames']),
                                   min(r['pose'][j] for r in original['frames']))

    def test_invalid_hold_cannot_enter_the_plan(self):
        for value in (-1,6,True,float('inf'),float('nan')):
            with self.assertRaises(SafetyError):
                rear_setback.compile_probe(self.config,20,hold_s=value)


if __name__=='__main__':
    unittest.main()
