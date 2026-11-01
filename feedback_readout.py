"""ADC-only baseline/noise acquisition. No relay/PCA9685 commands exist here."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import signal
import statistics
import time
import uuid

from feedback_hw import MuxADSReader,validate_layout


def acquisition(config,samples=50,*,reader_factory=MuxADSReader,raw_only=False):
    if type(samples) is not int or not 5<=samples<=1000:
        raise ValueError('samples must be an integer in 5..1000')
    reader=reader_factory(config)
    report={'schema_version':1,'mode':'ADC_RAW_REGISTER_BASELINE' if raw_only else 'ADC_ONLY_BASELINE',
            'raw_only':raw_only,'motion_performed':False,
            'motor_hardware_touched':False,'electrical_safety_confirmed':False,
            'config_sha256':hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest(),
            'started_at_utc':datetime.now(timezone.utc).isoformat(),'layout':config['devices'],
            'full_scale_v':config['full_scale_v'],'scans':[],'statistics':{},'cleanup_errors':[]}
    try:
        reader.open()
        for index in range(samples):
            report['scans'].append({'index':index,'readings':reader.scan(raw_only=raw_only)})
    except BaseException as exc:
        report['error']=type(exc).__name__+': '+str(exc)
    finally:
        reader.close()  # close/deselect BEFORE report serialization or disk writes
        report['cleanup_errors']=reader.cleanup_errors.copy()
    if report['scans']:
        for key in report['scans'][0]['readings']:
            rows=[r['readings'][key] for r in report['scans']]
            if raw_only:
                values=[r['register_signed16'] for r in rows]
                report['statistics'][key]={'n':len(values),'units':'signed16 register units; not ADC counts or volts',
                    'mean_register':statistics.mean(values),
                    'stdev_register':statistics.stdev(values) if len(values)>1 else None,
                    'min_register':min(values),'max_register':max(values),
                    'peak_to_peak_register':max(values)-min(values),
                    'format_mismatch_samples':sum(not r['format_not_contradicted'] for r in rows),
                    'model_verified':False,'volts':None}
                continue
            values=[r['volts'] for r in rows]
            median=statistics.median(values)
            report['statistics'][key]={'n':len(values),'mean_v':statistics.mean(values),
                'median_v':median,'stdev_v':statistics.stdev(values) if len(values)>1 else None,
                'mad_v':statistics.median(abs(v-median) for v in values),
                'min_v':min(values),'max_v':max(values),'peak_to_peak_v':max(values)-min(values),
                'saturated_samples':sum(r['saturated'] for r in rows),'lsb_v':rows[0]['lsb_v']}
            report['statistics'][key]['quality_flags']=(
                ['adc_saturated'] if report['statistics'][key]['saturated_samples'] else [])
    report['passed']=('error' not in report and not report['cleanup_errors'] and len(report['scans'])==samples)
    return report


def unresolved_proposal(config):
    return {'kind':'unverified_mapping_proposal','absolute_angle_calibration':False,
            'feedback_control_enabled':False,'motion_performed':False,
            'rows':[{'joint':joint,'pca_channel':channel,'mux_channel':3 if joint.startswith('RR_') else None,
                     'input':None,'confidence':'unidentified','status':'not_identified',
                     'source':'user wiring constraint only' if joint.startswith('RR_') else 'not measured'}
                    for joint,channel in config['pca9685_channels'].items()]}


def interrupt(*_): raise KeyboardInterrupt


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(Path(__file__).parent/'config'/'feedback-identification.json'))
    p.add_argument('--read',action='store_true',help='explicit real ADC acquisition; never motor output')
    p.add_argument('--i2c-idle-confirmed',action='store_true')
    p.add_argument('--samples',type=int,default=50)
    p.add_argument('--raw-only',action='store_true',help='explicit diagnostic: preserve registers, no voltage/type inference')
    p.add_argument('--output-dir',help='must be a new, non-existing directory')
    args=p.parse_args(argv)
    config=validate_layout(json.loads(Path(args.config).expanduser().read_text(encoding='utf-8-sig')))
    if not args.read:
        print('OFFLINE configuration OK. No I2C access. Voltage safety remains unverified.')
        print(json.dumps(unresolved_proposal(config),indent=2))
        return 0
    if not args.i2c_idle_confirmed:
        p.error('Stop all controller/scanner processes before using --read --i2c-idle-confirmed')
    if not 5<=args.samples<=1000: p.error('--samples must be in 5..1000')
    directory=Path(args.output_dir).expanduser() if args.output_dir else Path('reports')/(
        'feedback-baseline-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
    directory.mkdir(parents=True,exist_ok=False)  # disk/path failures happen before opening I2C
    handlers={}
    try:
        for name in ('SIGTERM','SIGHUP'):
            sig=getattr(signal,name,None)
            if sig is not None: handlers[sig]=signal.signal(sig,interrupt)
        report=acquisition(config,args.samples,raw_only=args.raw_only)
    finally:
        for sig,handler in handlers.items(): signal.signal(sig,handler)
    # No output hardware is open while results are saved.
    (directory/'baseline.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    (directory/'mapping-proposal.json').write_text(json.dumps(unresolved_proposal(config),indent=2),encoding='utf-8')
    print(json.dumps({'directory':str(directory.resolve()),'passed':report['passed'],
                      'error':report.get('error'),'cleanup_errors':report['cleanup_errors'],
                      'motion_performed':False,'electrical_safety_confirmed':False},indent=2))
    return 0 if report['passed'] else 2


if __name__=='__main__': raise SystemExit(main())
