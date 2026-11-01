import copy
import json
from pathlib import Path
import unittest

from feedback_preflight import electrical_blockers,identification_plan
from robot_config import JOINTS,SafetyError,load_config


class IdentificationPreflightTests(unittest.TestCase):
    def setUp(self):
        self.layout=json.loads((Path(__file__).resolve().parents[1]/'config'/'feedback-identification.json').read_text())
        self.robot=load_config()

    def test_unknown_measurements_block_and_no_motion_is_implemented(self):
        plan=identification_plan(self.layout,self.robot)
        self.assertTrue(plan['motion_blockers'])
        self.assertFalse(plan['hardware_execution_implemented'])
        self.assertFalse(plan['absolute_angle_calibration'])

    def test_pga_range_never_overrides_supply_limit(self):
        e=self.layout['electrical']
        e.update(adc_supply_v_measured={str(k):3.3 for k in range(4)},
                 feedback_min_v_measured={str(k):.1 for k in range(4)},
                 feedback_max_v_measured={str(k):4. for k in range(4)},
                 adaptation_circuit_verified=True,motion_electrically_verified=True,
                 measurement_evidence='SYNTHETIC TEST ONLY')
        self.assertTrue(any('GND..VDD' in x for x in electrical_blockers(self.layout)))
        e['feedback_max_v_measured']={str(k):3. for k in range(4)}
        self.assertEqual(electrical_blockers(self.layout),[])

    def test_each_target_changes_at_most_one_joint_and_always_returns_to_stand(self):
        before=copy.deepcopy(self.robot.data)
        plan=identification_plan(self.layout,self.robot,amplitude_deg=3.)
        stand=plan['stand_deg']
        for row in plan['targets']:
            changed=[j for j in JOINTS if row['target_deg'][j]!=stand[j]]
            self.assertIn(changed,([],[row['joint']]))
            self.assertLessEqual(abs(row['offset_deg']),3.)
            self.robot.validate_pose(row['target_deg'])
        self.assertEqual(plan['targets'][-1]['target_deg'],stand)
        self.assertEqual(self.robot.data,before)

    def test_invalid_amplitude_and_limit_violation_are_not_clipped(self):
        for amplitude in (0,3.1,True,float('nan')):
            with self.assertRaises((SafetyError,ValueError)):
                identification_plan(self.layout,self.robot,amplitude_deg=amplitude)
        self.robot.servos['FL_sh']['min_deg']=self.robot.pose('STAND')['FL_sh']-.5
        with self.assertRaises(SafetyError): identification_plan(self.layout,self.robot)

    def test_mapping_mismatch_is_not_fixed(self):
        self.robot.servos['FL_sh']['channel']=4
        with self.assertRaises(SafetyError): identification_plan(self.layout,self.robot)


if __name__=='__main__': unittest.main()
