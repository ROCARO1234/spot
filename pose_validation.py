"""Operator attestations bound to the exact configuration; never a sensor claim."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from robot_config import SafetyError, load_config


def fingerprint(config, pose):
    config.pose(pose)
    payload = {'robot': config.data, 'poses': config.poses, 'low_pose': pose}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def require_validation(config, pose, path):
    if pose in ('STAND', 'LIE', 'UP_READY'):
        raise SafetyError('Low pose must be a separately checked supported pose, not STAND/LIE/UP_READY')
    config.pose(pose)
    if not path:
        raise SafetyError('Low-pose hardware BLOCKED: a physical roundtrip validation record is required')
    try:
        record = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
        valid = (record['schema_version'] == 1 and record['pose'] == pose
                 and record['supported_roundtrip_confirmed'] is True
                 and record['fingerprint'] == fingerprint(config, pose)
                 and isinstance(record['operator'], str) and bool(record['operator'].strip())
                 and isinstance(record['evidence'], str) and bool(record['evidence'].strip())
                 and datetime.fromisoformat(record['recorded_at']).tzinfo is not None)
        if not valid:
            raise ValueError('record does not match current configuration or evidence is incomplete')
    except (OSError, ValueError, TypeError, KeyError) as exc:
        raise SafetyError(f'Low-pose hardware BLOCKED: {exc}') from exc
    return record


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['record'])
    p.add_argument('--config', required=True)
    p.add_argument('--poses')
    p.add_argument('--pose', required=True)
    p.add_argument('--operator', required=True)
    p.add_argument('--evidence', required=True)
    p.add_argument('--confirmed-supported-roundtrip', action='store_true', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args(argv)
    config = load_config(args.config, args.poses)
    if args.pose in ('STAND', 'LIE', 'UP_READY') or not args.operator.strip() or not args.evidence.strip():
        p.error('A safe low pose, operator and real physical evidence are required')
    record = {'schema_version': 1, 'pose': args.pose,
              'fingerprint': fingerprint(config, args.pose),
              'supported_roundtrip_confirmed': True, 'operator': args.operator,
              'evidence': args.evidence, 'recorded_at': datetime.now(timezone.utc).isoformat()}
    with Path(args.output).expanduser().open('x', encoding='utf-8') as f:
        json.dump(record, f, indent=2)
    print('Operator attestation recorded; this is not automatic physical verification.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
