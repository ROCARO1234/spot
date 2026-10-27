"""Supported, staged STAND -> fraction of low pose -> STAND commissioning.

Default is offline. Never marks a physical pose validated automatically.
"""
import argparse
import copy
import json
import signal
import time

from controller import MotionController
from robot_config import RobotConfig, SafetyError, finite_number, load_config
from robot_io import DryRunOutput, HardwareOutput


def probe_config(config, low_pose, fraction):
    finite_number(fraction, 'probe fraction', .01, 1.)
    if low_pose in ('STAND', 'LIE', 'UP_READY'):
        raise SafetyError('Probe requires a safe low pose within existing limits')
    stand, low = config.pose('STAND'), config.pose(low_pose)
    poses = copy.deepcopy(config.poses)
    poses['PROBE_LOW'] = {j: stand[j] + fraction*(low[j]-stand[j]) - config.servos[j]['offset_deg']
                          for j in stand}
    return RobotConfig(copy.deepcopy(config.data), poses)


def run_probe(config, *, hardware=False):
    controller = MotionController(config)
    output = HardwareOutput(config) if hardware else DryRunOutput(config)
    dt = 1/config.control['rate_hz']
    old_signals = {}
    previous = None
    def tick():
        nonlocal previous
        if not hardware:
            elapsed = dt
        else:
            now = time.monotonic()
            if previous is None:
                previous = now
            time.sleep(max(0., previous + dt - now))
            now = time.monotonic()
            elapsed = now - previous
            previous = now
            finite_number(elapsed, 'probe loop gap', 1e-6, config.control['max_loop_gap_s'])
        output.write(controller.update(elapsed), elapsed)
    try:
        if hardware:
            for name in ('SIGTERM', 'SIGHUP'):
                sig = getattr(signal, name, None)
                if sig is not None:
                    old_signals[sig] = signal.signal(sig, interrupt)
        output.open(controller.last_pose)
        for name in ('PROBE_LOW', 'STAND'):
            controller.request_pose(name)
            for _ in range(int(15/dt)):
                tick()
                if controller.settled and controller.posture == name:
                    break
            else:
                raise SafetyError('Probe transition deadline exceeded')
            # Hold only in a checked pose; signals still interrupt this loop.
            for _ in range(int(.5/dt)):
                tick()
    finally:
        output.close()
        for sig, previous in old_signals.items():
            signal.signal(sig, previous)
        if output.cleanup_errors:
            raise SafetyError('POWER-OFF ERROR: ' + '; '.join(output.cleanup_errors))


def interrupt(*_):
    raise KeyboardInterrupt


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', required=True)
    p.add_argument('--poses')
    p.add_argument('--low-pose', default='CROUCH_READY')
    p.add_argument('--fraction', type=float, default=.1)
    p.add_argument('--hardware', action='store_true')
    p.add_argument('--confirm-supported', action='store_true')
    args = p.parse_args(argv)
    try:
        config = probe_config(load_config(args.config, args.poses), args.low_pose, args.fraction)
        run_probe(config)
        print('Offline staged roundtrip: OK; no physical validation implied.', flush=True)
        if args.hardware:
            if not args.confirm_supported:
                raise SafetyError('Hardware probe requires explicit --confirm-supported and physical STAND alignment')
            print('HARDWARE PROBE: keep robot supported; Ctrl+C immediately cuts relay power.', flush=True)
            run_probe(config, hardware=True)
            print('Commands completed; relay OFF requested. Operator must inspect actual motion.')
        return 0
    except KeyboardInterrupt:
        print('Emergency stop: power-off requested; keep robot supported.')
        return 130
    except (SafetyError, OSError, ImportError, RuntimeError) as exc:
        print('STOPPED: ' + str(exc))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
