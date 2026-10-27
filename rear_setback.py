"""Supported commissioning of rear-foot prepositioning; default is offline.

Coordinates are predictions in the configured body frame, not measurements.
This is not a production loaded-rise controller or a ground-balance validation.
Default hardware probes require full external support. An explicit supervised
loaded-rise trial requires an exact, already-confirmed supported route record.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import time

import numpy as np

from gait_core import IKBinding, ease
from pose_validation import fingerprint, require_validation
from robot_config import JOINTS, SafetyError, finite_number, load_config
from robot_io import DryRunOutput, HardwareOutput, checked_frame


@dataclass
class Stage:
    name: str
    start: dict
    end: dict
    swing_leg: str | None = None

    def feet(self, u, lift):
        result = {leg: p + ease(u)*(self.end[leg]-p) for leg, p in self.start.items()}
        if self.swing_leg:
            result[self.swing_leg][2] += lift * 64*u**3*(1-u)**3
        return result


def compile_probe(config, setback_mm, *, stage='RL', fraction=1., hold_s=0.):
    """Whole validated frame list; never changes the supplied calibration."""
    finite_number(setback_mm, 'rear setback mm', 1., 30.)
    finite_number(fraction, 'probe fraction', .01, 1.)
    finite_number(hold_s, 'endpoint observation hold seconds', 0., 5.)
    if stage not in ('RL', 'RR', 'rise'):
        raise SafetyError('Stage must be RL, RR or rise')
    binding = IKBinding(config)
    low_pose = config.pose('CROUCH_READY')
    low = {leg: p[-1].copy() for leg, p in binding.forward(low_pose).items()}
    recovered = binding.solve(low)
    if max(abs(recovered[j]-low_pose[j]) for j in JOINTS) > 1e-6:
        raise SafetyError('Low-pose FK/IK branch mismatch; cannot construct a Cartesian probe')
    current = {leg: p.copy() for leg, p in low.items()}
    outbound = []
    for leg in (('RL', 'RR') if stage == 'rise' else (stage,)):
        target = {name: p.copy() for name, p in current.items()}
        target[leg][0] -= setback_mm * fraction / 1000
        outbound.append(Stage('SETBACK_' + leg, current, target, leg))
        current = target
    if stage == 'rise':
        target = {leg: p + fraction*(config.nominal[leg]-p) for leg, p in current.items()}
        outbound.append(Stage('RISE', current, target))
    stages = outbound + [Stage('RETURN_' + s.name, s.end, s.start, s.swing_leg)
                         for s in reversed(outbound)]
    dt = 1/config.control['rate_hz']
    lift = config.gait['lift_m'] * fraction
    # Validate the entire path before ANY output is opened. Lengthen timing
    # when needed; never clip a joint target or increase a configured limit.
    frames = [{'time_s': 0., 'phase': 'CROUCH_READY', 'pose': low_pose.copy(),
               'model_feet_body_m': {leg: p.tolist() for leg, p in low.items()}}]
    elapsed = 0.
    for segment in stages:
        count = math.ceil(config.control['pose_transition_s']/dt)
        for attempt in range(8):
            points = []
            previous = frames[-1]['pose']
            max_ratio = 0.
            for step in range(1, count+1):
                feet = segment.feet(step/count, lift)
                pose = binding.solve(feet)
                max_ratio = max(max_ratio, max(abs(pose[j]-previous[j]) /
                                    (config.servos[j]['max_rate_deg_s']*dt) for j in JOINTS))
                points.append((pose, feet))
                previous = pose
            if max_ratio <= 1.+1e-9:
                break
            count = math.ceil(count*max_ratio*1.05)
        else:
            raise SafetyError('Could not fit probe trajectory within the original slew limits')
        for pose, feet in points:
            checked_frame(config, pose, frames[-1]['pose'], dt)
            elapsed += dt
            frames.append({'time_s': elapsed, 'phase': segment.name, 'pose': pose,
                           'model_feet_body_m': {leg: p.tolist() for leg, p in feet.items()}})
        if not segment.name.startswith('RETURN_'):
            for _ in range(math.ceil(hold_s/dt)):
                elapsed += dt
                frames.append({'time_s': elapsed, 'phase': 'HOLD_'+segment.name,
                               'pose': frames[-1]['pose'].copy(),
                               'model_feet_body_m': {leg:p.tolist() for leg,p in segment.end.items()}})
    if max(abs(frames[-1]['pose'][j]-low_pose[j]) for j in JOINTS) > 1e-6:
        raise SafetyError('Probe does not return to the calibrated low pose')
    return {'schema_version': 1, 'config_fingerprint': fingerprint(config, 'CROUCH_READY'),
            'rear_setback_mm': setback_mm, 'fraction': fraction, 'stage': stage,
            'ground_validated': False, 'requires_full_external_support': True,
            'model_only': True, 'lift_mm': 1000*lift, 'duration_s': elapsed,
            'endpoint_observation_hold_s': hold_s,
            'frames': frames}


def summary(plan):
    return {k: v for k, v in plan.items() if k != 'frames'} | {'frames': len(plan['frames'])}


def route_fingerprint(plan):
    return hashlib.sha256(json.dumps({'config':plan['config_fingerprint'], 'frames':plan['frames']},
                                    sort_keys=True, allow_nan=False).encode()).hexdigest()


def record_supported_route(plan, path, operator, evidence):
    if plan['stage'] != 'rise' or plan['fraction'] != 1. or not operator.strip() or not evidence.strip():
        raise SafetyError('Recording requires a complete supported rise route and operator evidence')
    record={'schema_version':1,'supported_route_confirmed':True,
            'route_fingerprint':route_fingerprint(plan),'operator':operator,'evidence':evidence,
            'recorded_at':datetime.now(timezone.utc).isoformat(),'loaded_rise_validated':False}
    with Path(path).expanduser().open('x',encoding='utf-8') as f:
        json.dump(record,f,indent=2)


def require_supported_route(plan, path):
    if plan['stage'] != 'rise' or plan['fraction'] != 1. or not path:
        raise SafetyError('Loaded trial BLOCKED: a complete supported route record is required')
    try:
        r=json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
        if (r['schema_version'] != 1 or r['supported_route_confirmed'] is not True
                or r['route_fingerprint'] != route_fingerprint(plan)
                or not isinstance(r['operator'],str) or not r['operator'].strip()
                or not isinstance(r['evidence'],str) or not r['evidence'].strip()):
            raise ValueError('record mismatch or incomplete evidence')
    except (OSError,ValueError,KeyError,TypeError) as exc:
        raise SafetyError('Loaded trial BLOCKED: '+str(exc)) from exc
    return r


def run_probe(config, plan, *, hardware=False, log_path=None):
    if plan['config_fingerprint'] != fingerprint(config, 'CROUCH_READY'):
        raise SafetyError('Probe configuration changed')
    dt = 1/config.control['rate_hz']
    first = config.validate_pose(plan['frames'][0]['pose'])
    low = config.pose('CROUCH_READY')
    if max(abs(first[j]-low[j]) for j in JOINTS) > 1e-6:
        raise SafetyError('Probe must start in calibrated CROUCH_READY')
    previous_pose = first
    for frame in plan['frames'][1:]:
        previous_pose = checked_frame(config, frame['pose'], previous_pose, dt)
    if max(abs(previous_pose[j]-low[j]) for j in JOINTS) > 1e-6:
        raise SafetyError('Probe must return to CROUCH_READY')
    output = HardwareOutput(config) if hardware else DryRunOutput(config)
    log = None
    handlers = {}
    complete = False
    period = dt
    try:
        if log_path:
            log = Path(log_path).expanduser().open('x', encoding='utf-8', buffering=1)
            log.write(json.dumps({'event':'session','hardware':hardware,
                       'loaded_rise_trial':plan.get('loaded_rise_trial',False),
                       'requires_full_external_support':plan['requires_full_external_support'],
                       'full_support_during_repositioning':True,
                       'route_fingerprint':route_fingerprint(plan),
                       'rear_setback_mm':plan['rear_setback_mm'],'fraction':plan['fraction'],
                       'hold_s':plan['endpoint_observation_hold_s'],'measured_feedback':None})+'\n')
        if hardware:
            for name in ('SIGTERM', 'SIGHUP'):
                sig = getattr(signal, name, None)
                if sig is not None:
                    handlers[sig] = signal.signal(sig, interrupt)
        output.open(plan['frames'][0]['pose'])
        start = previous = time.monotonic()
        for frame in plan['frames'][1:]:
            dt = period
            if hardware:
                now = time.monotonic()
                time.sleep(max(0., previous+period-now))
                now = time.monotonic()
                dt = now-previous
                previous = now
                finite_number(dt, 'probe loop gap', 1e-6, config.control['max_loop_gap_s'])
            output.write(frame['pose'], dt)
            if log:
                log.write(json.dumps({'event': 'command', 'hardware': hardware,
                           'actual_elapsed_s': time.monotonic()-start if hardware else None,
                           'planned_time_s': frame['time_s'], 'phase': frame['phase'],
                           'commanded_deg': frame['pose'], 'measured_deg': None})+'\n')
        complete = True
    finally:
        try:
            output.close()  # emergency/exception never initiates a return trajectory
        finally:
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
            if log:
                try:
                    log.write(json.dumps({'event': 'shutdown', 'normal_complete': complete,
                                          'cleanup_errors': output.cleanup_errors})+'\n')
                finally:
                    log.close()
        if output.cleanup_errors:
            raise SafetyError('POWER-OFF ERROR: ' + '; '.join(output.cleanup_errors))


def interrupt(*_):
    raise KeyboardInterrupt


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--poses')
    p.add_argument('--rear-setback-mm', type=float, required=True)
    p.add_argument('--stage', choices=['RL', 'RR', 'rise'], default='RL')
    p.add_argument('--fraction', type=float, default=.25)
    p.add_argument('--hold-s', type=float, default=0., help='0..5 s stationary observation at each outbound endpoint')
    p.add_argument('--export', help='write the offline plan to a new JSON file')
    p.add_argument('--hardware', action='store_true')
    p.add_argument('--confirm-supported', action='store_true',
                   help='external support carries the FULL weight; physical CROUCH_READY alignment')
    p.add_argument('--low-validation')
    p.add_argument('--log')
    p.add_argument('--loaded-rise',action='store_true',help='explicit supervised load-transfer trial, no gait')
    p.add_argument('--confirm-catch-ready',action='store_true')
    p.add_argument('--supported-route-validation')
    p.add_argument('--record-supported-route',help='record already-observed supported route; offline only')
    p.add_argument('--operator')
    p.add_argument('--evidence')
    p.add_argument('--confirm-supported-route',action='store_true')
    args = p.parse_args(argv)
    try:
        config = load_config(args.config, args.poses)
        plan = compile_probe(config, args.rear_setback_mm, stage=args.stage,
                             fraction=args.fraction, hold_s=args.hold_s)
        run_probe(config, plan)  # deterministic validation with dry output
        print(json.dumps(summary(plan), indent=2), flush=True)
        if args.export:
            with Path(args.export).expanduser().open('x', encoding='utf-8') as f:
                json.dump(plan, f)
        if args.record_supported_route:
            if args.hardware or not args.confirm_supported_route or not args.operator or not args.evidence:
                raise SafetyError('Route attestation is offline-only and requires explicit operator evidence')
            record_supported_route(plan,args.record_supported_route,args.operator,args.evidence)
            print('Supported route attestation recorded; loaded rise is NOT validated.')
        if args.hardware:
            if not args.log:
                raise SafetyError('Hardware commissioning requires --log')
            if args.loaded_rise:
                if not args.confirm_catch_ready or args.confirm_supported:
                    raise SafetyError('Loaded trial requires explicit --confirm-catch-ready and separate support mode')
                require_supported_route(plan,args.supported_route_validation)
            elif not args.confirm_supported:
                raise SafetyError('Hardware commissioning requires --confirm-supported and --log')
            require_validation(config, 'CROUCH_READY', args.low_validation)
            print('LOADED RISE TRIAL: full support during foot repositioning; catch ready during extension/return.'
                  if args.loaded_rise else 'SUPPORTED COMMISSIONING ONLY: full external support; no balance guarantee.', flush=True)
            if args.loaded_rise:
                plan = dict(plan,requires_full_external_support=False,loaded_rise_trial=True,
                            full_support_during_repositioning=True,ground_validated=False)
            run_probe(config, plan, hardware=True, log_path=args.log)
            print('Probe completed; relay OFF requested. Physical observation is required.')
        return 0
    except KeyboardInterrupt:
        print('Emergency stop: relay OFF requested; no programmed return.')
        return 130
    except (SafetyError, OSError, ImportError, RuntimeError) as exc:
        print('STOPPED: '+str(exc))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
