#!/usr/bin/env python3
"""
Replay recorded two-view observations through the tracker to measure static drift.

Input is the observation log written by the hardware verification scripts
(``evidence/hardware_20261124_dualview/dual_verify7.json`` has ``observations``
with ``camera``, ``label``, ``position`` (base frame), ``depth_m``, ``score``,
``valid_depth_fraction``, ``t`` (receive, relative seconds) and ``stamp``
(absolute capture seconds)).

The replayed measurement covariance is rebuilt from ``config/cameras.yaml``
because the log does not carry it; positions, labels and capture times are the
recorded ones. Two modes:

  fixed   current tracker: global per-view correction + per-track view offset +
          velocity from same-camera differences + capped extrapolation.
  legacy  pre-2026-11-24 behaviour: no bias model, full-gain cross-view
          updates, 1 m/s velocity prior and 1 s extrapolation. Used only as the
          A/B baseline; it is not the production code.

Report per track: max/p95 displacement from the settled-window median, final
offset, maximum published speed, learned view offset. Exit code 1 if a static
track in ``--require`` labels exceeds ``--limit`` (default 20 mm) in fixed mode.

Example:

    python3 scripts/replay_static_drift.py \
        --source evidence/hardware_20261124_dualview/dual_verify7.json \
        --out evidence/analysis_20261124_drift_bias/replay_fixed.json

    python3 scripts/replay_static_drift.py --mode legacy --out .../legacy.json
"""

import argparse
import datetime
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
CORE = REPO_ROOT / 'ros2_ws' / 'src' / 'jcheng_perception_core'
if str(CORE) not in sys.path:
    sys.path.insert(0, str(CORE))

from jcheng_perception_core import (  # noqa: E402
    Phase,
    Role,
    TargetObservation,
    TrackManager,
    TrackStatus,
    ViewBiasModel,
    fuse_observations,
    load_camera_specs,
    load_fusion_config,
    phase_for_observations,
)


def _observation(row, specs, fusion, view_bias, clock_offset_s):
    """Rebuild a TargetObservation from one recorded row."""
    spec = specs[row['camera']]
    depth_m = float(row['depth_m'])
    valid_fraction = float(row.get('valid_depth_fraction', 1.0))
    occlusion = max(0.0, 1.0 - min(
        1.0, valid_fraction / max(0.5, 2.0 * spec.min_valid_depth_fraction)))
    intrinsics = spec.intrinsics
    sigma_x = depth_m * spec.pixel_sigma_px / float(intrinsics[0, 0])
    sigma_y = depth_m * spec.pixel_sigma_px / float(intrinsics[1, 1])
    sigma_z = (
        spec.depth_noise_quadratic * depth_m ** 2 + spec.depth_noise_floor_m)
    covariance = np.diag(
        [sigma_x ** 2, sigma_y ** 2, sigma_z ** 2]) + np.eye(3) * (
            fusion.measurement_floor_m ** 2)
    observation = TargetObservation(
        camera_id=str(row['camera']),
        role=spec.role,
        label=str(row['label']),
        class_id=0,
        capture_ns=int(round(
            (float(row['stamp']) - clock_offset_s) * 1e9)),
        received_ns=int(round(float(row['t']) * 1e9)),
        frame_seq=0,
        frame_id_camera='',
        detection_score=float(row['score']),
        keypoint_score=0.0,
        position_base=np.asarray(row['position'], dtype=np.float64),
        covariance_base=covariance,
        depth_m=depth_m,
        valid_depth_fraction=valid_fraction,
        occlusion_ratio=occlusion,
        pixel=(0.0, 0.0),
    )
    return view_bias.correct(observation)


def _coalesce(rows, coalesce_ms):
    """Group observations by receive time into coalesced batches."""
    batches = []
    for row in sorted(rows, key=lambda item: item['t']):
        batches.append({
            'deadline_ns': int(round(
                (float(row['t']) + coalesce_ms / 1000.0) * 1e9)),
            'rows': [row],
        })
    return batches


class LegacyTrackManager(TrackManager):
    """Pre-fix tracker, kept only to produce the drift A/B baseline."""

    def apply_group(self, track, observations, phase, now_ns):
        fused = fuse_observations(observations, phase, self.specs, self.fusion)
        if fused is None:
            return None
        for observation in observations:
            observation.view_weight = float(
                fused.weights.get(observation.camera_id, 0.0))
        dt = min(
            0.5,
            max(0.0, (int(fused.stamp_ns) - int(track.last_state_ns)) / 1e9))
        track.kf.predict(dt)
        track.kf.update(fused.position_base, fused.covariance_base)
        track.last_state_ns = int(fused.stamp_ns)
        track.last_update_ns = int(now_ns)
        track.hits += 1
        if Role.WRIST in [
                self.specs[camera_id].role
                for camera_id in fused.sources if camera_id in self.specs]:
            track.wrist_hits += 1
        if Role.EXTERNAL in [
                self.specs[camera_id].role
                for camera_id in fused.sources if camera_id in self.specs]:
            track.external_hits += 1
        track.last_fused = fused
        if fused.orientation_base is not None:
            track.last_orientation = np.asarray(
                fused.orientation_base, dtype=np.float64).copy()
            track.last_orientation_sigma = float(fused.orientation_sigma_rad)
        if track.hits >= self.fusion.confirm_hits:
            track.status = TrackStatus.CONFIRMED
        return fused


def replay(rows, specs, fusion, view_bias, mode, publish_hz):
    """Run one replay and return an annotated target log."""
    if mode == 'legacy':
        manager = LegacyTrackManager(specs, fusion)
        fusion.max_extrapolation_s = 1.0
    else:
        manager = TrackManager(specs, fusion)
    clock_offset_s = max(
        float(row['stamp']) - float(row['t']) for row in rows)
    pending = _coalesce(rows, fusion.coalesce_ms)
    tick = 0.0
    step = 1.0 / publish_hz
    log = []
    phase = Phase.SEARCH
    last_t = max(float(row['t']) for row in rows) + 1.0
    while tick <= last_t:
        now_ns = int(round(tick * 1e9))
        due = [batch for batch in pending if batch['deadline_ns'] <= now_ns]
        if due:
            pending = [batch for batch in pending
                       if batch['deadline_ns'] > now_ns]
            observations = [
                _observation(row, specs, fusion, view_bias, clock_offset_s)
                for batch in due for row in batch['rows']]
            if observations:
                found = phase_for_observations(
                    observations, list(manager.tracks.values()), fusion)
                phase = found
                assignments = manager.associate(observations, now_ns)
                manager.update(assignments, phase, now_ns)
            manager.merge_duplicates(now_ns)
        manager.expire(now_ns)
        for target in manager.targets(now_ns, phase):
            log.append({
                't': tick,
                'target_id': target.target_id,
                'position': np.asarray(
                    target.position_base, dtype=np.float64),
                'label': str(target.label),
                'sources': list(target.source_cameras),
            })
        tick += step
    return manager, log


def summarize(manager, log, settle_s, min_samples=20):
    """Return per-track motion metrics and the learned view offset."""
    tracks = {}
    grouped = {}
    for sample in log:
        grouped.setdefault(sample['target_id'], []).append(sample)
    for target_id, samples in grouped.items():
        if len(samples) < min_samples:
            continue
        times = np.asarray([sample['t'] for sample in samples])
        if times[-1] - times[0] < 1.5:
            continue
        positions = np.asarray([sample['position'] for sample in samples])
        settled_mask = times >= times[0] + settle_s
        settled = positions[settled_mask]
        if settled.shape[0] < min_samples:
            settled = positions
        centre = np.median(settled, axis=0)
        displacement = np.linalg.norm(settled - centre, axis=1)
        dt = np.diff(times)
        velocity = np.linalg.norm(np.diff(positions, axis=0), axis=1) / np.maximum(dt, 1e-6)
        track = manager.tracks.get(target_id) if manager is not None else None
        tracks[target_id] = {
            'label': samples[-1]['label'],
            'samples': len(samples),
            't_first': float(times[0]),
            't_last': float(times[-1]),
            'median_mm': np.round(centre * 1000.0, 2).tolist(),
            'max_displacement_mm': float(displacement.max() * 1000.0),
            'p95_displacement_mm': float(
                np.percentile(displacement, 95) * 1000.0),
            'final_offset_mm': float(
                np.linalg.norm(positions[-1] - centre) * 1000.0),
            'max_speed_mm_s': float(velocity.max() * 1000.0),
            'view_offset_hits': int(
                getattr(track, 'view_offset_hits', 0) if track else 0),
            'view_offset_mm': np.round(
                getattr(track, 'view_offset', np.zeros(3)) * 1000.0,
                2).tolist() if track else [0.0, 0.0, 0.0],
            'sources_last': samples[-1]['sources'],
        }
    return tracks


def _print_table(tracks):
    """Print a compact markdown table sorted by maximum displacement."""
    header = ('| target | samples | max disp mm | p95 mm | final mm | '
              'max speed mm/s | view offset (mm) |')
    print(header)
    print('|---|---:|---:|---:|---:|---:|---|')
    for target_id, stats in sorted(
            tracks.items(),
            key=lambda item: item[1]['max_displacement_mm'],
            reverse=True):
        print('| {} | {} | {:.1f} | {:.1f} | {:.1f} | {:.0f} | {} |'.format(
            target_id, stats['samples'], stats['max_displacement_mm'],
            stats['p95_displacement_mm'], stats['final_offset_mm'],
            stats['max_speed_mm_s'], stats['view_offset_mm']))


def main(argv=None):
    """Parse arguments, run one replay and write the report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--source', default=str(
            REPO_ROOT / 'evidence' / 'hardware_20261124_dualview'
            / 'dual_verify7.json'))
    parser.add_argument(
        '--fusion', default=str(REPO_ROOT / 'config' / 'fusion_dual_hardware.yaml'))
    parser.add_argument(
        '--bias', default=str(REPO_ROOT / 'config' / 'view_bias.yaml'))
    parser.add_argument(
        '--mode', choices=('fixed', 'legacy'), default='fixed')
    parser.add_argument('--publish-hz', type=float, default=30.0)
    parser.add_argument('--settle-s', type=float, default=1.0)
    parser.add_argument('--limit-mm', type=float, default=20.0)
    parser.add_argument(
        '--require', default='cup,dining table,chair,person',
        help='comma-separated labels that must stay under --limit-mm')
    parser.add_argument('--out', default='')
    parser.add_argument(
        '--recorded-targets', action='store_true',
        help='skip the replay and measure the targets already in the file '
             '(the pre-fix hardware baseline)')
    args = parser.parse_args(argv)

    document = json.loads(Path(args.source).read_text(encoding='utf-8'))
    specs = load_camera_specs(str(REPO_ROOT / 'config' / 'cameras.yaml'))
    fusion = load_fusion_config(args.fusion)
    required = [name.strip() for name in args.require.split(',') if name.strip()]
    if args.recorded_targets:
        recorded = document.get('targets')
        if not recorded:
            raise SystemExit('{}: no recorded targets'.format(args.source))
        log = []
        for record in recorded:
            for target in record.get('targets', []):
                log.append({
                    't': float(record['t']),
                    'target_id': str(target['target_id']),
                    'position': np.asarray(
                        target['position'], dtype=np.float64),
                    'label': str(target.get('label', '')),
                    'sources': list(target.get('source_cameras', [])),
                })
        manager = None
        mode = 'recorded'
    else:
        rows = document.get('observations')
        if not rows:
            raise SystemExit('{}: no observations'.format(args.source))
        mode = args.mode
        if mode == 'legacy':
            fusion.initial_velocity_sigma_mps = 1.0
            view_bias = ViewBiasModel.empty()
        else:
            view_bias = ViewBiasModel.from_config(args.bias)
        manager, log = replay(
            rows, specs, fusion, view_bias, mode, args.publish_hz)
    tracks = summarize(manager, log, args.settle_s)
    print('== {} mode, {} =='.format(mode, Path(args.source).name))
    _print_table(tracks)

    violations = []
    if mode == 'fixed':
        for target_id, stats in tracks.items():
            if stats['label'] in required and \
                    stats['max_displacement_mm'] > args.limit_mm:
                violations.append(target_id)
    worst = max(
        (stats['max_displacement_mm'] for stats in tracks.values()
         if stats['label'] in required), default=0.0)
    print('\nrequired-label worst max displacement: {:.1f} mm '
          '(limit {:.1f} mm)'.format(worst, args.limit_mm))
    if violations:
        print('FAIL: {}'.format(', '.join(sorted(violations))))
    elif mode == 'fixed':
        print('PASS')

    if args.out:
        report = {
            'generated_utc': datetime.datetime.now(
                datetime.timezone.utc).isoformat(timespec='seconds'),
            'source': str(Path(args.source).resolve()),
            'mode': mode,
            'fusion': str(Path(args.fusion).resolve()),
            'bias': str(Path(args.bias).resolve()) if mode == 'fixed' else '',
            'coalesce_ms': float(fusion.coalesce_ms),
            'publish_hz': args.publish_hz,
            'settle_s': args.settle_s,
            'limit_mm': args.limit_mm,
            'required_labels': required,
            'worst_required_mm': float(worst),
            'violations': sorted(violations),
            'tracks': tracks,
        }
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
        print('wrote {}'.format(args.out))
    return 1 if violations else 0


if __name__ == '__main__':
    sys.exit(main())
