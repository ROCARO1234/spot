"""Offline candidates after the failed loaded rise; never imports hardware."""
import argparse
import copy
import json
from pathlib import Path

from gait_core import CrawlPlanner, IKBinding
from rear_setback import compile_probe, summary as probe_summary
from robot_config import RobotConfig, load_config
from simulation import simulate, summary


def compare(config):
    binding = IKBinding(config)
    low = {leg: points[-1] for leg, points in binding.forward(config.pose('CROUCH_READY')).items()}
    result = {'hardware_exercised': False, 'ground_validated': False,
              'rear_setback_operator_estimate_mm': [20,30],
              'model_low_feet_body_mm': {leg:(p*1000).tolist() for leg,p in low.items()},
              'model_rear_forward_of_stand_mm': {
                  leg: float(1000*(low[leg][0]-config.nominal[leg][0])) for leg in ('RL','RR')},
              'notes': ['Body-frame FK predictions only; the low-pose foot heights differ and the body attitude is not measured.',
                        'External support must carry the full weight during preparatory commissioning.',
                        'No calibration file, servo limit, stride cap, speed or production default is changed.'],
              'rear_candidates': [], 'gait_candidates': []}
    for mm in (20,25,30):
        try:
            plan = compile_probe(config,mm,stage='rise',fraction=1.)
            result['rear_candidates'].append(probe_summary(plan) | {'offline_passed':True})
        except ValueError as exc:
            result['rear_candidates'].append({'rear_setback_mm':mm,'offline_passed':False,'error':str(exc)})
    for lift_mm in (12,18):
        data=copy.deepcopy(config.data)
        data['gait']['lift_m']=lift_mm/1000
        candidate=RobotConfig(data,copy.deepcopy(config.poses))
        cycle=CrawlPlanner(candidate,dynamicity=.5).cycle_time
        try:
            records=simulate(candidate,(.003,0.,0.),2*cycle,dynamicity=.5)
            first=[]
            first_leg=None
            for r in records:
                if r['swing']:
                    first_leg=first_leg or r['swing'][0]
                    if r['swing'][0]!=first_leg and first:
                        break
                    first.append(r)
                elif first:
                    break
            travel=max(r['feet'][first_leg][0] for r in first)-min(r['feet'][first_leg][0] for r in first)
            peak=max(r['feet'][first_leg][2]-candidate.nominal[first_leg][2] for r in first)
            result['gait_candidates'].append(summary(records) | {
                'lift_mm':lift_mm,'dynamicity':.5,'cycle_time_s':cycle,
                'requested_vx_mps':.003,'stride_cap_mm':1000*config.gait['max_stride_m'],
                'first_model_swing_x_range_mm':1000*float(travel),
                'first_model_swing_peak_lift_mm':1000*float(peak),
                'offline_passed':True,'hardware_validated':False})
        except ValueError as exc:
            result['gait_candidates'].append({'lift_mm':lift_mm,'offline_passed':False,'error':str(exc)})
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True)
    p.add_argument('--poses')
    p.add_argument('--output',required=True)
    args=p.parse_args(argv)
    result=compare(load_config(args.config,args.poses))
    with Path(args.output).expanduser().open('x',encoding='utf-8') as f:
        json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
    return 0 if all(x['offline_passed'] for x in result['rear_candidates']+result['gait_candidates']) else 2


if __name__=='__main__':
    raise SystemExit(main())
