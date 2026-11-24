#!/usr/bin/env python3
"""Summarise fixed-point CSVs from ``fixed_point_check.py``.

One CSV per camera. Each row is one sample with two estimates of the same
physical point (production ``depth`` path and RGB-only ``pnp``), plus the
camera origin. Rows are grouped by ``pose_seq`` (the check script starts a new
sequence when the camera origin jumps > 10 mm); groups that are too short or
where the camera moved are dropped as travel, not poses.

Reports, per pose, the median point and within-pose spread; then the pairwise
agreement between poses (the hand-eye test) and, when an external CSV is given,
the cross-view agreement (the extrinsic test). The exit code is 1 when any
kept pair exceeds ``--limit`` (default 5 mm).

Example:

    python3 fixed_point_report.py --wrist /tmp/fixed_point_wrist.csv \
        --ext /tmp/fixed_point_ext.csv --out /tmp/report.json
"""

import argparse
import csv
import json
import sys

import numpy as np

COLUMNS = {
    'depth': ('p_base_x', 'p_base_y', 'p_base_z'),
    'pnp': ('pnp_base_x', 'pnp_base_y', 'pnp_base_z'),
}
CAMERA_COLUMNS = ('cam_x', 'cam_y', 'cam_z')


def _load(path):
    """Return a list of sample dicts; numeric columns converted to float."""
    rows = []
    with open(path, newline='', encoding='utf-8') as handle:
        for row in csv.DictReader(handle):
            sample = {}
            try:
                for key, value in row.items():
                    if key == 'tf_mode':
                        sample[key] = value
                    else:
                        sample[key] = float(value)
            except (TypeError, ValueError):
                continue
            sample['pose_seq'] = int(sample['pose_seq'])
            rows.append(sample)
    return rows


def _segment(rows, min_samples, min_duration_s):
    """Group by pose_seq and keep only settled poses."""
    groups = {}
    for row in rows:
        groups.setdefault(row['pose_seq'], []).append(row)
    kept = []
    for sequence in sorted(groups):
        samples = groups[sequence]
        if len(samples) < min_samples:
            continue
        duration = (samples[-1]['t_ns'] - samples[0]['t_ns']) / 1e9
        if duration < min_duration_s:
            continue
        origins = np.array(
            [[row[key] for key in CAMERA_COLUMNS] for row in samples])
        if float(np.linalg.norm(origins.std(axis=0))) > 0.005:
            continue
        depth = np.array([[row[key] for key in COLUMNS['depth']]
                          for row in samples])
        pnp = np.array([[row[key] for key in COLUMNS['pnp']]
                        for row in samples])
        kept.append({
            'seq': sequence,
            'n': len(samples),
            'duration_s': round(duration, 1),
            'camera': origins.mean(axis=0),
            'depth': depth,
            'pnp': pnp,
            'depth_median': np.median(depth, axis=0),
            'pnp_median': np.median(pnp, axis=0),
            'depth_std': depth.std(axis=0),
            'pnp_std': pnp.std(axis=0),
        })
    return kept


def _fmt(point):
    """Format a 3-vector in millimetres."""
    return '({:+.1f}, {:+.1f}, {:+.1f})'.format(*(np.asarray(point) * 1000.0))


def _pairs(poses, limit):
    """Print pairwise agreement; return the largest kept-pair norm."""
    worst = 0.0
    for index, first in enumerate(poses):
        for second in poses[index + 1:]:
            for name in ('depth', 'pnp'):
                delta = second[name + '_median'] - first[name + '_median']
                norm = float(np.linalg.norm(delta))
                worst = max(worst, norm)
                flag = 'PASS' if norm <= limit else 'FAIL'
                print('  pair seq {}/{} {:4s} delta={} |d|={:.2f} mm  {}'.format(
                    first['seq'], second['seq'], name, _fmt(delta),
                    norm * 1000.0, flag))
    return worst


def main(argv=None):
    """Load, segment and report; exit 1 when the limit is exceeded."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wrist', required=True)
    parser.add_argument('--ext', default='')
    parser.add_argument('--limit', type=float, default=0.005)
    parser.add_argument('--min-samples', type=int, default=10)
    parser.add_argument('--min-duration', type=float, default=3.0)
    parser.add_argument('--out', default='')
    args = parser.parse_args(argv)

    wrist = _segment(
        _load(args.wrist), args.min_samples, args.min_duration)
    if not wrist:
        print('no settled wrist poses (need >= {} samples and >= {:.0f} s '
              'with a stable camera)'.format(
                  args.min_samples, args.min_duration))
        return 2

    print('wrist poses kept: {}'.format(len(wrist)))
    for pose in wrist:
        print(
            'pose {}  n={}  {:.1f} s  cam={}  '
            'depth={} (sd {:.2f} mm)  pnp={} (sd {:.2f} mm)'.format(
                pose['seq'], pose['n'], pose['duration_s'],
                _fmt(pose['camera']), _fmt(pose['depth_median']),
                float(np.linalg.norm(pose['depth_std'])) * 1000.0,
                _fmt(pose['pnp_median']),
                float(np.linalg.norm(pose['pnp_std'])) * 1000.0))

    print('wrist pairwise agreement (limit {:.1f} mm):'.format(
        args.limit * 1000.0))
    worst = _pairs(wrist, args.limit)

    ext_pose = None
    if args.ext:
        ext = _segment(_load(args.ext), args.min_samples, args.min_duration)
        if not ext:
            print('no settled external segment')
        else:
            ext_pose = ext[0]
            print('external pose  n={}  {:.1f} s  depth={}  pnp={}'.format(
                ext_pose['n'], ext_pose['duration_s'],
                _fmt(ext_pose['depth_median']),
                _fmt(ext_pose['pnp_median'])))
            print('cross-view agreement (limit {:.1f} mm):'.format(
                args.limit * 1000.0))
            for pose in wrist:
                for name in ('depth', 'pnp'):
                    delta = pose[name + '_median'] - ext_pose[name + '_median']
                    norm = float(np.linalg.norm(delta))
                    worst = max(worst, norm)
                    flag = 'PASS' if norm <= args.limit else 'FAIL'
                    print('  wrist seq {} vs ext {:4s} delta={} '
                          '|d|={:.2f} mm  {}'.format(
                              pose['seq'], name, _fmt(delta),
                              norm * 1000.0, flag))

    passed = worst <= args.limit
    print('worst agreement {:.2f} mm -> {}'.format(
        worst * 1000.0, 'PASS' if passed else 'FAIL'))
    if args.out:
        summary = {
            'limit_mm': args.limit * 1000.0,
            'worst_mm': worst * 1000.0,
            'pass': passed,
            'wrist_poses': [
                {
                    'seq': pose['seq'],
                    'n': pose['n'],
                    'duration_s': pose['duration_s'],
                    'camera_m': pose['camera'].tolist(),
                    'depth_median_m': pose['depth_median'].tolist(),
                    'pnp_median_m': pose['pnp_median'].tolist(),
                    'depth_std_mm': float(
                        np.linalg.norm(pose['depth_std'])) * 1000.0,
                    'pnp_std_mm': float(
                        np.linalg.norm(pose['pnp_std'])) * 1000.0,
                }
                for pose in wrist
            ],
        }
        if ext_pose is not None:
            summary['external'] = {
                'n': ext_pose['n'],
                'depth_median_m': ext_pose['depth_median'].tolist(),
                'pnp_median_m': ext_pose['pnp_median'].tolist(),
            }
        with open(args.out, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
