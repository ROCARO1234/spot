"""Guarded independent identification executor; not the unavailable supplied script.

No gait, autonomous rise or feedback control. Default mode is offline. A real
run requires physical electrical evidence, supported STAND alignment and a
strict ADC-format preflight before any power-on attempt.
"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import signal
import time

from feedback_hw import MuxADSReader,finite
from feedback_mapping import ADC_KEYS,propose_mapping
from feedback_preflight import identification_plan
from robot_config import JOINTS,SafetyError,load_config
from robot_io import HardwareOutput


def identify(layout,robot,*,confirmed_supported=False,amplitude_deg=1.,rate_deg_s=1.,
             repeats=3,samples_per_window=5,output_factory=HardwareOutput,
             reader_factory=MuxADSReader,clock=time.monotonic,sleep=time.sleep):
    report={'schema_version':1,'kind':'servo_feedback_identification_trial',
            'started_at_utc':datetime.now(timezone.utc).isoformat(),
            'power_on_attempted':False,'powered_baseline_collected':False,
            'absolute_angle_calibration':False,'feedback_control_enabled':False,
            'trials':[],'baseline_v':None,'cleanup_errors':[],'proposal':None}
    reader=output=None
    try:
        plan=identification_plan(layout,robot,amplitude_deg=amplitude_deg,
                                  rate_deg_s=rate_deg_s,repeats=repeats)
        report['preflight']={k:v for k,v in plan.items() if k!='targets'}
        if not confirmed_supported: raise SafetyError('Physical supported STAND readiness is required')
        if plan['motion_blockers']: raise SafetyError('Motion blocked: '+'; '.join(plan['motion_blockers']))
        if type(samples_per_window) is not int or not 3<=samples_per_window<=20:
            raise SafetyError('Require 3..20 samples per settled window')
        reader=reader_factory(layout)
        reader.open()
        # This scan checks protocol/format only. It is NOT position feedback or
        # the powered noise baseline and is never used to infer a joint map.
        prepower=reader.scan()
        if set(prepower)!=set(ADC_KEYS): raise SafetyError('Pre-power scan missing ADC inputs')
        report['prepower_format_scan']=prepower
        for value in prepower.values():
            finite(value['volts'],'pre-power ADC conversion')
        output=output_factory(robot)
        report['power_on_attempted']=True
        current=plan['stand_deg'].copy()
        output.open(current)  # stages all 12 STAND channels while relay is OFF, then powers ON
        previous_write=clock()
        period=1/robot.control['rate_hz']

        def write(pose):
            nonlocal previous_write,current
            now=clock(); dt=now-previous_write
            if not 1e-6<=dt<=robot.control['max_loop_gap_s']:
                raise SafetyError('Identification control-loop gap exceeded its limit')
            output.write(pose,dt)
            previous_write=now; current=pose.copy()

        def hold(seconds):
            end=clock()+seconds
            while clock()+1e-9<end:
                sleep(period)
                write(current)

        def move(target):
            start_pose=current.copy()
            distance=max(abs(target[j]-current[j]) for j in JOINTS)
            if distance<1e-12: return
            duration=distance/rate_deg_s
            start=clock()
            while True:
                sleep(period)
                u=min(1.,(clock()-start)/duration)
                pose={j:start_pose[j]+u*(target[j]-start_pose[j]) for j in JOINTS}
                write(pose)
                if u>=1.: return

        def window():
            values={key:[] for key in ADC_KEYS}; scans=[]
            for _ in range(samples_per_window):
                scan=reader.scan()
                if set(scan)!=set(ADC_KEYS): raise SafetyError('Settled scan missing ADC inputs')
                for key,value in scan.items():
                    volts=finite(value['volts'],'powered ADC volts')
                    lsb=finite(value['lsb_v'],'ADC LSB')
                    supply=layout['electrical']['adc_supply_v_measured'][key.split(':')[0]]
                    if value['saturated'] or volts < -2*lsb or volts>supply:
                        raise SafetyError('Powered ADC out of expected electrical range or saturated: '+key)
                    values[key].append(volts)
                scans.append(scan)
                write(current)  # holding still does not waive the real loop-gap limit
            return values,scans

        hold(.25)
        report['baseline_v'],report['powered_baseline_scans']=window()
        report['powered_baseline_collected']=True
        for target in plan['targets']:
            move(target['target_deg'])
            hold(.2)
            values,scans=window()
            report['trials'].append({'joint':target['joint'],'repeat':target['repeat'],
                                    'offset_deg':target['offset_deg'],'readings_v':values,
                                    'scans':scans,'commanded_deg':current.copy()})
    except BaseException as exc:
        report['error']=type(exc).__name__+': '+str(exc)
    finally:
        # Power-off first, before ADC cleanup, analysis or report serialization.
        if output is not None:
            try: output.close()
            except BaseException as exc: report['cleanup_errors'].append('Power-off exception: '+str(exc))
            report['cleanup_errors'].extend(output.cleanup_errors)
        if reader is not None:
            try: reader.close()
            except BaseException as exc: report['cleanup_errors'].append('ADC cleanup exception: '+str(exc))
            report['cleanup_errors'].extend(reader.cleanup_errors)
    if 'error' not in report and not report['cleanup_errors']:
        trials=[{k:r[k] for k in ('joint','repeat','offset_deg','readings_v')} for r in report['trials']]
        quantization={f'{d["mux_channel"]}:{ain}':layout['full_scale_v']/
                      (2048 if d['model']=='ADS1015' else 32768)
                      for d in layout['devices'] for ain in range(4)}
        try:
            report['proposal']=propose_mapping(trials,report['baseline_v'],layout['pca9685_channels'],
                                               quantization_v=quantization)
        except Exception as exc: report['error']='Analysis failed: '+str(exc)
    report['completed_without_error']='error' not in report and not report['cleanup_errors']
    return report


def interrupt(*_): raise KeyboardInterrupt


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(Path(__file__).parent/'config'/'feedback-identification.json'))
    p.add_argument('--robot-config',required=True); p.add_argument('--poses',required=True)
    p.add_argument('--hardware',action='store_true'); p.add_argument('--confirm-supported',action='store_true')
    p.add_argument('--i2c-idle-confirmed',action='store_true')
    p.add_argument('--amplitude-deg',type=float,default=1.); p.add_argument('--rate-deg-s',type=float,default=1.)
    p.add_argument('--repeats',type=int,default=3); p.add_argument('--samples-per-window',type=int,default=5)
    p.add_argument('--output-dir',required=True,help='new directory; prior results are never overwritten')
    a=p.parse_args(argv)
    layout=json.loads(Path(a.config).expanduser().read_text(encoding='utf-8-sig'))
    robot=load_config(a.robot_config,a.poses)
    directory=Path(a.output_dir).expanduser(); directory.mkdir(parents=True,exist_ok=False)
    if not a.hardware:
        report=identification_plan(layout,robot,amplitude_deg=a.amplitude_deg,
                                   rate_deg_s=a.rate_deg_s,repeats=a.repeats)
        report['hardware_was_opened']=False
        (directory/'offline-plan.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({'hardware_was_opened':False,'blockers':report['motion_blockers']},indent=2))
        return 2 if report['motion_blockers'] else 0
    if not a.confirm_supported or not a.i2c_idle_confirmed:
        p.error('--hardware requires supported STAND readiness and stopped competing I2C processes')
    handlers={}
    try:
        for name in ('SIGTERM','SIGHUP'):
            sig=getattr(signal,name,None)
            if sig is not None: handlers[sig]=signal.signal(sig,interrupt)
        report=identify(layout,robot,confirmed_supported=True,amplitude_deg=a.amplitude_deg,
                        rate_deg_s=a.rate_deg_s,repeats=a.repeats,samples_per_window=a.samples_per_window)
    finally:
        for sig,handler in handlers.items(): signal.signal(sig,handler)
    # identify() has already attempted power-off and closed all hardware.
    (directory/'trial-report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    if report['proposal'] is not None:
        (directory/'mapping-proposal.json').write_text(json.dumps(report['proposal'],indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({'power_on_attempted':report['power_on_attempted'],
                      'powered_baseline_collected':report['powered_baseline_collected'],
                      'completed_without_error':report['completed_without_error'],
                      'all_joints_proposed':bool(report['proposal'] and report['proposal']['all_joints_proposed']),
                      'error':report.get('error'),'cleanup_errors':report['cleanup_errors'],
                      'output_dir':str(directory.resolve())},indent=2))
    if not report['completed_without_error']: return 2
    return 0 if report['proposal']['all_joints_proposed'] else 3


if __name__=='__main__': raise SystemExit(main())
