"""Non-executable bounded identification plan and electrical motion blockers."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from feedback_hw import EXPECTED_PCA,finite,validate_layout
from robot_config import JOINTS,SafetyError,load_config


def electrical_blockers(layout):
    e=layout.get('electrical',{})
    issues=[]
    supplies=e.get('adc_supply_v_measured')
    mins=e.get('feedback_min_v_measured'); maxs=e.get('feedback_max_v_measured')
    def complete(value): return isinstance(value,dict) and set(value)=={'0','1','2','3'}
    if not all(complete(v) for v in (supplies,mins,maxs)):
        issues.append('Supply and feedback extrema must be physically measured for each mux channel 0..3')
    else:
        for key in ('0','1','2','3'):
            try:
                supply=finite(supplies[key],'ADC supply'); lo=finite(mins[key],'feedback min')
                hi=finite(maxs[key],'feedback max')
                if not 2.<=supply<=5.5 or not 0.<=lo<=hi<=supply:
                    issues.append(f'mux{key}: feedback must remain GND..VDD; PGA is not an input-voltage allowance')
                if hi>=layout['full_scale_v']:
                    issues.append(f'mux{key}: measured maximum reaches/exceeds configured ADC scale')
            except ValueError as exc: issues.append(f'mux{key}: {exc}')
    if e.get('adaptation_circuit_verified') is not True:
        issues.append('Feedback adaptation/direct wiring and I2C voltage levels are not physically verified')
    if e.get('motion_electrically_verified') is not True:
        issues.append('Electrical motion review has not been explicitly confirmed')
    if not isinstance(e.get('measurement_evidence'),str) or not e['measurement_evidence'].strip():
        issues.append('Physical measurement evidence is missing')
    return issues


def identification_plan(layout,robot,*,amplitude_deg=1.,rate_deg_s=1.,repeats=3):
    validate_layout(layout)
    amplitude=finite(amplitude_deg,'amplitude'); rate=finite(rate_deg_s,'probe speed')
    if not 0.<amplitude<=3.: raise SafetyError('Identification amplitude must be >0 and <=3 degrees; never auto-increased')
    if not 0.<rate<=3.: raise SafetyError('Identification probe speed must be >0 and <=3 deg/s')
    if type(repeats) is not int or not 2<=repeats<=5: raise SafetyError('Require 2..5 repeated symmetric probes')
    if {j:robot.servos[j]['channel'] for j in JOINTS}!=EXPECTED_PCA:
        raise SafetyError('Installed calibrated PCA mapping does not match the supplied wiring')
    h=robot.hardware; r=h['relay']
    if (h['pca9685_address']!=0x40 or (r['bus'],r['address'],r['register'],r['on_data'],r['off_data'])
            !=(1,0x08,1,[2],[1])):
        raise SafetyError('Installed relay/PCA configuration differs; no inferred correction')
    stand=robot.pose('STAND')
    targets=[]
    for joint in JOINTS:
        if rate>robot.servos[joint]['max_rate_deg_s']:
            raise SafetyError('Probe speed exceeds calibrated joint limit')
        for repeat in range(repeats):
            for offset in (0.,amplitude,0.,-amplitude,0.):
                frame=stand.copy(); frame[joint]+=offset
                frame=robot.validate_pose(frame)  # all endpoints checked before ANY possible future arm
                targets.append({'joint':joint,'repeat':repeat,'offset_deg':offset,
                                'pca_channel':EXPECTED_PCA[joint],'target_deg':frame,
                                'rate_limit_deg_s':rate})
    return {'kind':'non_executable_identification_plan','hardware_execution_implemented':False,
            'amplitude_deg':amplitude,'rate_deg_s':rate,'repeats':repeats,'initial_pose':'STAND',
            'initial_power_can_command_all_12_servos':True,'all_targets_within_calibrated_limits':True,
            'motion_blockers':robot.hardware_blockers()+electrical_blockers(layout),
            'operator_readiness_still_required':True,'absolute_angle_calibration':False,
            'feedback_control_enabled':False,'stand_deg':stand,'targets':targets}


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(Path(__file__).parent/'config'/'feedback-identification.json'))
    p.add_argument('--robot-config',required=True)
    p.add_argument('--poses',required=True)
    p.add_argument('--amplitude-deg',type=float,default=1.)
    p.add_argument('--rate-deg-s',type=float,default=1.)
    p.add_argument('--output',required=True,help='new file only; no motor commands exist in this script')
    args=p.parse_args(argv)
    layout=json.loads(Path(args.config).expanduser().read_text(encoding='utf-8-sig'))
    plan=identification_plan(layout,load_config(args.robot_config,args.poses),
                             amplitude_deg=args.amplitude_deg,rate_deg_s=args.rate_deg_s)
    with Path(args.output).expanduser().open('x',encoding='utf-8') as f: json.dump(plan,f,indent=2)
    print(json.dumps({k:v for k,v in plan.items() if k not in ('targets','stand_deg')},indent=2))
    return 2 if plan['motion_blockers'] else 0


if __name__=='__main__': raise SystemExit(main())
