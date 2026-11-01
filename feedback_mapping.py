"""Pure offline association proposals; no hardware access or angle calibration.

Each trial is a settled window for one commanded offset. ADC keys are ``mux:AIN``.
Confidence is ordinal evidence strength, never a probability or physical angle
accuracy. In particular, V/commanded-degree slopes do not calibrate real angles.
"""
from __future__ import annotations

import math
from statistics import mean, median, stdev


LEGS = ('FL', 'FR', 'RL', 'RR')
JOINTS = tuple(f'{leg}_{part}' for leg in LEGS for part in ('sh', 'up', 'lo'))
ADC_KEYS = tuple(f'{mux}:{ain}' for mux in range(4) for ain in range(4))


def _finite(value, label, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{label}: finite numeric value required')
    if low is not None and value < low or high is not None and value > high:
        raise ValueError(f'{label}: outside allowed range')
    return float(value)


def _window(values, label):
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        raise ValueError(f'{label}: at least three real samples are required')
    values = [_finite(x, label) for x in values]
    return {'mean_v': mean(values), 'stddev_v': stdev(values), 'n': len(values)}


def _candidate(key, repeat_records, baseline, quantization, min_snr):
    contrasts, slopes, noises, midpoint_errors, zero_drifts = [], [], [], [], []
    for repeat in repeat_records:
        plus, minus, zeros = repeat['plus'], repeat['minus'], repeat['zeros']
        p, m = plus['readings_v'][key], minus['readings_v'][key]
        z = [row['readings_v'][key] for row in zeros]
        contrast = p['mean_v']-m['mean_v']
        contrasts.append(contrast)
        slopes.append(contrast/(plus['offset_deg']-minus['offset_deg']))
        noises.append(max(baseline['stddev_v'], p['stddev_v'], m['stddev_v'],
                          *(row['stddev_v'] for row in z), quantization))
        midpoint_errors.append(abs((p['mean_v']+m['mean_v'])/2-mean(row['mean_v'] for row in z)))
        zero_drifts.append(max(row['mean_v'] for row in z)-min(row['mean_v'] for row in z))
    response = median(abs(x) for x in contrasts)
    noise = max(noises)
    signs_agree = all(x > 0 for x in contrasts) or all(x < 0 for x in contrasts)
    magnitudes = [abs(x) for x in contrasts]
    variation = (max(magnitudes)-min(magnitudes))/response if response else None
    return {'adc': key, 'response_v': response, 'minimum_repeat_response_v': min(magnitudes),
            'noise_floor_v': noise,
            'snr': min(abs(contrast)/noise_floor for contrast,noise_floor in zip(contrasts,noises)),
            'repeat_contrasts_v': contrasts,
            'commanded_degree_slope_v_per_deg_proxy': median(slopes),
            'repeat_sign_consistent': signs_agree, 'relative_repeat_spread': variation,
            'max_zero_window_drift_v': max(zero_drifts),
            'max_reference_midpoint_error_v': max(midpoint_errors),
            'reference_consistent': all(error <= max(min_snr*noise, .25*response)
                                        for error in midpoint_errors+zero_drifts)}


def propose_mapping(trials, baseline, servo_channels, *, min_response_v=.015,
                    min_snr=6., uniqueness_ratio=2., max_repeat_spread=.25,
                    quantization_v=.003, min_repeats=2):
    """Return all 12 rows, with null mapping/confidence for every rejected row.

    ``trials`` rows: joint, repeat (nonnegative int), offset_deg (within +/-3),
    readings_v (all 16 ADC keys -> >=3 samples). Each repeat needs exactly one
    positive/negative window of equal amplitude and at least one zero window.
    ``baseline`` contains >=3 samples for every ADC. It is used for noise, not
    absolute reference voltage (motor power may change the reference voltage).
    ``quantization_v`` may be one conservative LSB floor or a complete key map.
    No hardware type is inferred from ADC noise or apparent resolution.
    """
    min_response_v = _finite(min_response_v, 'min_response_v', 1e-9)
    min_snr = _finite(min_snr, 'min_snr', 1.)
    uniqueness_ratio = _finite(uniqueness_ratio, 'uniqueness_ratio', 1.000001)
    max_repeat_spread = _finite(max_repeat_spread, 'max_repeat_spread', 0., 1.)
    if isinstance(min_repeats, bool) or not isinstance(min_repeats, int) or min_repeats < 2:
        raise ValueError('min_repeats must be at least two')
    if not isinstance(servo_channels, dict) or set(servo_channels) != set(JOINTS):
        raise ValueError('Exactly the 12 joint-to-PCA9685 channels are required')
    channels = list(servo_channels.values())
    if any(isinstance(c, bool) or not isinstance(c, int) or not 0 <= c <= 15 for c in channels) or len(set(channels)) != 12:
        raise ValueError('PCA9685 channels must be unique integers in 0..15')
    if not isinstance(baseline, dict) or set(baseline) != set(ADC_KEYS):
        raise ValueError('Baseline must contain all 16 explicitly named ADC inputs')
    baseline_stats = {key: _window(baseline[key], f'baseline {key}') for key in ADC_KEYS}
    if isinstance(quantization_v, dict):
        if set(quantization_v) != set(ADC_KEYS):
            raise ValueError('Quantization floor must name all 16 inputs')
        floors = {key: _finite(quantization_v[key], 'quantization_v', 1e-12) for key in ADC_KEYS}
    else:
        floors = {key: _finite(quantization_v, 'quantization_v', 1e-12) for key in ADC_KEYS}
    grouped = {joint: {} for joint in JOINTS}
    if not isinstance(trials, list):
        raise ValueError('Trials must be an explicit list of settled measurement windows')
    for raw in trials:
        if not isinstance(raw, dict) or set(raw) != {'joint', 'repeat', 'offset_deg', 'readings_v'}:
            raise ValueError('Malformed trial schema')
        joint, repeat = raw['joint'], raw['repeat']
        if joint not in grouped or isinstance(repeat, bool) or not isinstance(repeat, int) or repeat < 0:
            raise ValueError('Unknown joint or invalid repeat index')
        offset = _finite(raw['offset_deg'], 'offset_deg', -3., 3.)
        readings = raw['readings_v']
        if not isinstance(readings, dict) or set(readings) != set(ADC_KEYS):
            raise ValueError('Every trial must measure all 16 inputs; no missing-input fallback')
        record = {'offset_deg': offset, 'readings_v': {
            key: _window(readings[key], f'{joint}/{repeat}/{key}') for key in ADC_KEYS}}
        grouped[joint].setdefault(repeat, []).append(record)
    rows = []
    for joint in JOINTS:
        row = {'joint': joint, 'pca_channel': servo_channels[joint], 'mux_channel': None,
               'input': None, 'confidence': None, 'status': 'rejected', 'reasons': [],
               'evidence': {'baseline': baseline_stats, 'repeats': len(grouped[joint])}}
        repeats = []
        for records in grouped[joint].values():
            plus = [r for r in records if r['offset_deg'] > 0]
            minus = [r for r in records if r['offset_deg'] < 0]
            zeros = [r for r in records if r['offset_deg'] == 0]
            if (len(plus) != 1 or len(minus) != 1 or not zeros
                    or abs(plus[0]['offset_deg']+minus[0]['offset_deg']) > 1e-9):
                row['reasons'].append('incomplete_or_asymmetric_repeat')
                continue
            repeats.append({'plus': plus[0], 'minus': minus[0], 'zeros': zeros})
        if len(repeats) < min_repeats or row['reasons']:
            row['reasons'].append('insufficient_complete_repeats')
            rows.append(row)
            continue
        ranked = sorted((_candidate(key, repeats, baseline_stats[key], floors[key], min_snr)
                         for key in ADC_KEYS), key=lambda c: (-c['response_v'], c['adc']))
        best, second = ranked[:2]
        ratio = best['response_v']/second['response_v'] if second['response_v'] else None
        row['evidence'].update({'ranked_candidates': ranked, 'winner_to_runnerup_ratio': ratio,
                                'complete_repeats': len(repeats)})
        if best['minimum_repeat_response_v'] < min_response_v:
            row['reasons'].append('response_too_small')
        if best['snr'] < min_snr:
            row['reasons'].append('too_noisy')
        if not best['repeat_sign_consistent']:
            row['reasons'].append('repeat_direction_inconsistent')
        if best['relative_repeat_spread'] is None or best['relative_repeat_spread'] > max_repeat_spread:
            row['reasons'].append('repeat_magnitude_inconsistent')
        if not best['reference_consistent']:
            row['reasons'].append('reference_drift_or_nonlinearity')
        if second['response_v'] > 0 and best['response_v'] < uniqueness_ratio*second['response_v']:
            row['reasons'].append('ambiguous_or_crosstalk')
        mux, ain = map(int, best['adc'].split(':'))
        if (joint.startswith('RR_') and mux != 3) or (not joint.startswith('RR_') and mux == 3):
            row['reasons'].append('incompatible_RR_mux_partition')
        if not row['reasons']:
            row.update(mux_channel=mux, input=ain, status='proposal',
                       confidence='strong' if best['snr'] >= 2*min_snr and len(repeats) >= 3
                       and (ratio is None or ratio >= 2*uniqueness_ratio) else 'moderate')
        rows.append(row)

    def reject(row, reason):
        if reason not in row['reasons']:
            row['reasons'].append(reason)
        row.update(mux_channel=None, input=None, confidence=None, status='rejected')

    # Never switch to a weaker channel to conceal collisions.
    winners = {}
    for row in rows:
        if row['status'] == 'proposal':
            winners.setdefault((row['mux_channel'], row['input']), []).append(row)
    for collision in winners.values():
        if len(collision) > 1:
            for row in collision:
                reject(row, 'duplicate_ADC_assignment')
    leg_rows = {leg: [row for row in rows if row['joint'].startswith(leg+'_')] for leg in LEGS}
    leg_muxes = {}
    for leg, members in leg_rows.items():
        accepted = [row for row in members if row['status'] == 'proposal']
        muxes = {row['mux_channel'] for row in accepted}
        if len(accepted) != 3 or len(muxes) != 1:
            for row in members:
                reject(row, 'incomplete_or_conflicting_leg_partition')
        else:
            leg_muxes[leg] = next(iter(muxes))
    for leg, mux in leg_muxes.items():
        if sum(other == mux for other in leg_muxes.values()) > 1:
            for row in leg_rows[leg]:
                reject(row, 'mux_shared_by_multiple_legs')
    return {'schema_version': 1, 'kind': 'unverified_mapping_proposal',
            'absolute_angle_calibration': False, 'feedback_control_enabled': False,
            'confidence_is_probability': False, 'rows': rows,
            'all_joints_proposed': all(row['status'] == 'proposal' for row in rows),
            'thresholds': {'min_response_v': min_response_v, 'min_snr': min_snr,
                           'uniqueness_ratio': uniqueness_ratio, 'max_repeat_spread': max_repeat_spread,
                           'min_repeats': min_repeats, 'quantization_floor_v': floors},
            'limitations': ['Commanded-angle slope is only a proxy, not measured physical angle.',
                            'Settled repeated windows cannot rule out every temporal drift or mechanical coupling.',
                            'Operator physical verification is required before accepting this proposal.']}
