"""Synthetic fixtures exercise rejection; they are not real sensor readings."""
import copy
import math
import unittest

from feedback_mapping import ADC_KEYS, JOINTS, propose_mapping


PCA = dict(zip(JOINTS, (15,14,13,3,2,1,11,10,9,7,6,5)))
ASSIGNMENTS = {joint: f'{leg}:{part}' for leg, prefix in enumerate(('FL','FR','RL','RR'))
               for part, joint in enumerate(f'{prefix}_{p}' for p in ('sh','up','lo'))}


def fixture(repeats=3, assignments=None):
    assignments = assignments or ASSIGNMENTS
    baseline = {key:[1.499,1.5,1.501,1.5] for key in ADC_KEYS}
    trials = []
    for joint in JOINTS:
        for repeat in range(repeats):
            for offset in (0., -1., 1., 0.):
                readings = {key:[1.5]*4 for key in ADC_KEYS}
                key = assignments[joint]
                direction = -1 if joint.endswith('_lo') else 1
                readings[key] = [1.5+direction*.12*offset+noise for noise in (-.001,0,.001,0)]
                trials.append({'joint':joint,'repeat':repeat,'offset_deg':offset,'readings_v':readings})
    return trials, baseline


def indexed(result):
    return {row['joint']:row for row in result['rows']}


class MappingTests(unittest.TestCase):
    def test_clean_repeated_proposal_and_proxy_labels(self):
        trials, baseline = fixture()
        original = copy.deepcopy(trials)
        result = propose_mapping(trials,baseline,PCA)
        self.assertTrue(result['all_joints_proposed'])
        self.assertFalse(result['absolute_angle_calibration'])
        self.assertFalse(result['feedback_control_enabled'])
        self.assertFalse(result['confidence_is_probability'])
        for row in result['rows']:
            self.assertEqual(f"{row['mux_channel']}:{row['input']}",ASSIGNMENTS[row['joint']])
            self.assertEqual(row['confidence'],'strong')
            self.assertEqual(row['pca_channel'],PCA[row['joint']])
            self.assertTrue(row['evidence']['ranked_candidates'][0]['repeat_sign_consistent'])
        self.assertEqual(trials, original)

    def test_equal_responses_are_ambiguous_and_never_arbitrarily_selected(self):
        trials, baseline = fixture()
        for trial in trials:
            if trial['joint']=='FL_sh':
                trial['readings_v']['0:3'] = list(trial['readings_v']['0:0'])
        row = indexed(propose_mapping(trials,baseline,PCA))['FL_sh']
        self.assertIn('ambiguous_or_crosstalk',row['reasons'])
        self.assertIsNone(row['mux_channel'])
        self.assertIsNone(row['confidence'])

    def test_duplicate_winners_are_rejected_without_runnerup_assignment(self):
        assignments = dict(ASSIGNMENTS,FL_up='0:0')
        trials, baseline = fixture(assignments=assignments)
        rows = indexed(propose_mapping(trials,baseline,PCA))
        for joint in ('FL_sh','FL_up'):
            self.assertIn('duplicate_ADC_assignment',rows[joint]['reasons'])
            self.assertIsNone(rows[joint]['input'])

    def test_crosstalk_competing_input_rejects_even_when_primary_is_stronger(self):
        trials, baseline = fixture()
        for trial in trials:
            if trial['joint']=='FR_sh':
                trial['readings_v']['2:3'] = [1.5+.1*trial['offset_deg']]*4
        row = indexed(propose_mapping(trials,baseline,PCA))['FR_sh']
        self.assertIn('ambiguous_or_crosstalk',row['reasons'])

    def test_RR_is_bound_to_mux3_and_other_legs_cannot_use_it(self):
        assignments = dict(ASSIGNMENTS,RR_sh='0:3',FL_up='3:3')
        trials, baseline = fixture(assignments=assignments)
        rows = indexed(propose_mapping(trials,baseline,PCA))
        for joint in ('RR_sh','FL_up'):
            self.assertIn('incompatible_RR_mux_partition',rows[joint]['reasons'])

    def test_conflicting_leg_mux_and_shared_leg_mux_rejected(self):
        for assignments in (dict(ASSIGNMENTS,FL_up='1:3'),
                            dict(ASSIGNMENTS,FR_sh='0:1',FR_up='0:2',FR_lo='0:3')):
            trials, baseline = fixture(assignments=assignments)
            result = propose_mapping(trials,baseline,PCA)
            self.assertFalse(result['all_joints_proposed'])
            self.assertTrue(all(row['mux_channel'] is None for row in result['rows']
                                if row['joint'].startswith('FL_')))

    def test_noisy_baseline_and_noisy_trial_windows_rejected(self):
        for baseline_noise in (True,False):
            trials, baseline = fixture()
            if baseline_noise:
                baseline['2:0'] = [.9,1.3,1.7,2.1]
            else:
                for trial in trials:
                    if trial['joint']=='RL_sh':
                        trial['readings_v']['2:0'] = [trial['readings_v']['2:0'][0]+v
                                                      for v in (-.3,-.1,.1,.3)]
            row = indexed(propose_mapping(trials,baseline,PCA))['RL_sh']
            self.assertIn('too_noisy',row['reasons'])

    def test_repeat_reversal_or_inconsistent_magnitude_rejected(self):
        for multiplier, reason in ((-1,'repeat_direction_inconsistent'),
                                   (2,'repeat_magnitude_inconsistent')):
            trials, baseline = fixture()
            for trial in trials:
                if trial['joint']=='RL_up' and trial['repeat']==2:
                    trial['readings_v']['2:1'] = [1.5+multiplier*.12*trial['offset_deg']]*4
            row = indexed(propose_mapping(trials,baseline,PCA))['RL_up']
            self.assertIn(reason,row['reasons'])

    def test_missing_repeats_missing_zero_and_asymmetric_offsets_reject(self):
        trials, baseline = fixture(repeats=1)
        self.assertTrue(all(r['status']=='rejected' for r in propose_mapping(trials,baseline,PCA)['rows']))
        for variant in ('zero','negative','asymmetric'):
            trials, baseline = fixture()
            if variant=='zero':
                trials=[r for r in trials if not (r['joint']=='FL_sh' and r['offset_deg']==0)]
            elif variant=='negative':
                trials=[r for r in trials if not (r['joint']=='FL_sh' and r['repeat']==1 and r['offset_deg']<0)]
            else:
                for row in trials:
                    if row['joint']=='FL_sh' and row['repeat']==1 and row['offset_deg']>0:
                        row['offset_deg']=2.
            row = indexed(propose_mapping(trials,baseline,PCA))['FL_sh']
            self.assertIn('insufficient_complete_repeats',row['reasons'])

    def test_zero_reference_drift_rejected(self):
        trials, baseline = fixture()
        for row in trials:
            if row['joint']=='FR_lo' and row['offset_deg']==0:
                row['readings_v']['1:2']=[2.]*4
        row=indexed(propose_mapping(trials,baseline,PCA))['FR_lo']
        self.assertIn('reference_drift_or_nonlinearity',row['reasons'])

    def test_malformed_or_missing_measurements_never_become_zero_fallback(self):
        trials, baseline = fixture()
        for value in (math.nan,math.inf,True,'1.2'):
            broken=copy.deepcopy(trials)
            broken[0]['readings_v']['0:0'][0]=value
            with self.assertRaises(ValueError):
                propose_mapping(broken,baseline,PCA)
        broken=copy.deepcopy(trials); del broken[0]['readings_v']['1:3']
        with self.assertRaises(ValueError):
            propose_mapping(broken,baseline,PCA)
        broken_baseline=copy.deepcopy(baseline); del broken_baseline['3:3']
        with self.assertRaises(ValueError):
            propose_mapping(trials,broken_baseline,PCA)

    def test_quantization_floor_and_response_threshold_are_enforced(self):
        trials, baseline = fixture()
        rows=indexed(propose_mapping(trials,baseline,PCA,quantization_v=.1))
        self.assertIn('too_noisy',rows['FL_sh']['reasons'])
        rows=indexed(propose_mapping(trials,baseline,PCA,min_response_v=.3))
        self.assertIn('response_too_small',rows['FL_sh']['reasons'])
        floors={key:.003 if not key.startswith('3:') else .0001875 for key in ADC_KEYS}
        self.assertTrue(propose_mapping(trials,baseline,PCA,quantization_v=floors)['all_joints_proposed'])

    def test_one_weak_repeat_cannot_hide_behind_a_strong_median(self):
        trials, baseline=fixture()
        for trial in trials:
            if trial['joint']=='FL_sh' and trial['repeat']==2:
                trial['readings_v']['0:0']=[1.5+.1*trial['offset_deg']]*4
        row=indexed(propose_mapping(trials,baseline,PCA,min_response_v=.21))['FL_sh']
        self.assertIn('response_too_small',row['reasons'])


if __name__=='__main__':
    unittest.main()
