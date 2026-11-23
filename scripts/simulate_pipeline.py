#!/usr/bin/env python3
"""
Offline end-to-end simulation of the distributed perception design.

It runs the real core pipeline (timestamp rings, depth back-projection,
association, distance-weighted fusion, Kalman tracking) against a moving
tabletop target seen by the measured chassis D435i pose and a moving wrist
camera. Detections are delayed by a configurable GPU latency, an external
occlusion window is injected, and a drop-recheck phase is exercised at the end.

Usage:
    python3 scripts/simulate_pipeline.py
    python3 scripts/simulate_pipeline.py --duration 12 --json
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
CORE = REPO_ROOT / 'ros2_ws' / 'src' / 'jcheng_perception_core'
if str(CORE) not in sys.path:
    sys.path.insert(0, str(CORE))

from jcheng_perception_core import (  # noqa: E402
    Detection,
    DepthFrame,
    FrameMeta,
    Keypoint,
    PerceptionPipeline,
    Phase,
    build_observation,
    load_camera_specs,
    load_fusion_config,
)

TABLE_Z = -0.0825
WIDTH = 640
HEIGHT = 400
K = np.array([
    [500.0, 0.0, WIDTH / 2.0],
    [0.0, 500.0, HEIGHT / 2.0],
    [0.0, 0.0, 1.0],
])
EXT_TRANSLATION = np.array(
    [0.1393005143144151, 0.027041016800109785, 0.03674937694821427])
EXT_ROTATION = np.array([
    [-0.04216838436036936, -0.5377799793511505, 0.8420300001600372],
    [-0.9986357296162156, 0.04866561522702682, -0.018929802652546318],
    [-0.030797839117794233, -0.8416794827626792, -0.5390984524203566],
])
WRIST_ROTATION = np.diag([1.0, -1.0, -1.0])


def transform(translation, rotation):
    """Return a 4x4 base<-camera transform."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def project(camera_matrix, camera_transform, point_base):
    """Project a base-frame point into a camera frame."""
    point_camera = (point_base - camera_transform[:3, 3]) @ \
        camera_transform[:3, :3]
    if point_camera[2] <= 0.02:
        return None, None
    intrinsics = camera_matrix
    u = intrinsics[0, 0] * point_camera[0] / point_camera[2] + intrinsics[0, 2]
    v = intrinsics[1, 1] * point_camera[1] / point_camera[2] + intrinsics[1, 2]
    if not (0.0 <= u < WIDTH and 0.0 <= v < HEIGHT):
        return None, None
    return (u, v), float(point_camera[2])


def make_depth(depth_m, rng, noise_m):
    """Return a uint16 depth image with Gaussian noise."""
    metres = depth_m + rng.normal(0.0, noise_m, size=(HEIGHT, WIDTH))
    millimetres = np.clip(metres * 1000.0, 1.0, 65535.0)
    return millimetres.astype(np.uint16)


def detection_at(u, v, size=34.0, label='target'):
    """Return a detection box centred on a pixel."""
    half = size / 2.0
    return Detection(
        label=label,
        score=0.94,
        bbox_xyxy=(u - half, v - half, u + half, v + half),
        class_id=0,
        keypoints=[Keypoint(u, v, 0.92)],
    )


def stats(errors):
    """Return a compact error summary."""
    if not errors:
        return {'n': 0}
    values = np.asarray(errors, dtype=np.float64)
    return {
        'n': int(values.size),
        'rmse_m': float(np.sqrt(np.mean(values ** 2))),
        'mean_m': float(np.mean(values)),
        'p95_m': float(np.percentile(values, 95)),
        'max_m': float(np.max(values)),
    }


def run(duration_s, seed=11, gpu_latency_ms=30.0):
    """Run the simulation and return a metrics dictionary."""
    specs = load_camera_specs(str(REPO_ROOT / 'config' / 'cameras.yaml'))
    for spec in specs.values():
        spec.depth_noise_quadratic = 0.0025
        spec.depth_noise_floor_m = 0.0015
    fusion = load_fusion_config(str(REPO_ROOT / 'config' / 'fusion_demo.yaml'))
    fusion.coalesce_ms = 20.0
    pipeline = PerceptionPipeline(specs, fusion, {})
    rng = np.random.default_rng(seed)
    rate_hz = 10.0
    period_ns = int(1e9 / rate_hz)
    latency_ns = int(gpu_latency_ms * 1e6)
    flush_delay_ns = latency_ns + int(fusion.coalesce_ms * 1e6) + 5_000_000
    ext_transform = transform(EXT_TRANSLATION, EXT_ROTATION)
    warmup_s = 1.5
    occlusion_start_s = 4.0
    occlusion_end_s = 5.0
    fused_errors = []
    external_errors = []
    wrist_errors = []
    occlusion_errors = []
    phases = {}
    targets_present = 0
    frames = 0

    def truth_at(elapsed):
        return np.array([
            0.35 + 0.030 * math.sin(0.40 * elapsed),
            0.02 + 0.020 * math.sin(0.27 * elapsed + 0.4),
            TABLE_Z,
        ])

    for index in range(int(duration_s * rate_hz)):
        elapsed = index / rate_hz
        capture_ns = 1_000_000_000 + index * period_ns
        flush_ns = capture_ns + flush_delay_ns
        truth = truth_at(elapsed)
        wrist_translation = np.array([
            0.35 + 0.020 * math.sin(0.20 * elapsed),
            0.02,
            0.28,
        ])
        wrist_transform = transform(wrist_translation, WRIST_ROTATION)
        pose_by_camera = {
            'd435i_ext': ext_transform,
            'aurora_wrist': wrist_transform,
        }
        detections = {}
        for camera_id, camera_transform in pose_by_camera.items():
            pixel, depth_m = project(K, camera_transform, truth)
            if pixel is None:
                continue
            depth = DepthFrame(
                data=make_depth(depth_m, rng, 0.0006),
                scale_m=0.001,
                stamp_ns=capture_ns,
            )
            frame = FrameMeta(
                camera_id=camera_id,
                seq=index,
                capture_ns=capture_ns,
                received_ns=capture_ns + 1_000_000,
                width=WIDTH,
                height=HEIGHT,
                frame_id=camera_id,
                camera_matrix=K,
                distortion=np.zeros(5),
            )
            pipeline.ingest_frame(
                camera_id, index, capture_ns, capture_ns + 1_000_000,
                WIDTH, HEIGHT, camera_id, K, np.zeros(5))
            pipeline.ingest_depth(
                camera_id, capture_ns, depth.data, scale_m=0.001)
            pipeline.ingest_pose(camera_id, capture_ns, camera_transform)
            observation = build_observation(
                detection_at(*pixel), frame, depth, camera_transform,
                specs[camera_id], fusion)
            if observation is not None:
                error = float(np.linalg.norm(
                    observation.position_base - truth))
                if camera_id == 'd435i_ext':
                    external_errors.append(error)
                else:
                    wrist_errors.append(error)
            occluded = occlusion_start_s <= elapsed < occlusion_end_s
            if camera_id == 'd435i_ext' and occluded:
                continue
            detections[camera_id] = detection_at(*pixel)
        for camera_id, detection in detections.items():
            pipeline.ingest_detections(
                camera_id, index, capture_ns, capture_ns + latency_ns,
                [detection])
        targets = pipeline.flush(flush_ns)
        frames += 1
        phases[pipeline.last_phase.name] = \
            phases.get(pipeline.last_phase.name, 0) + 1
        if targets:
            targets_present += 1
            if elapsed >= warmup_s:
                error = float(np.linalg.norm(
                    targets[0].position_base - truth_at(
                        elapsed + flush_delay_ns / 1e9)))
                fused_errors.append(error)
                if occlusion_start_s <= elapsed < occlusion_end_s:
                    occlusion_errors.append(error)

    recheck = None
    if pipeline.tracks.tracks:
        now_ns = 1_000_000_000 + int(duration_s * 1e9)
        pipeline.set_phase_override(Phase.DROP_RECHECK, now_ns)
        capture_ns = now_ns + period_ns
        truth = truth_at(duration_s + 0.1)
        for camera_id, camera_transform in pose_by_camera.items():
            pixel, depth_m = project(K, camera_transform, truth)
            if pixel is None:
                continue
            pipeline.ingest_frame(
                camera_id, 999, capture_ns, capture_ns + 1_000_000,
                WIDTH, HEIGHT, camera_id, K, np.zeros(5))
            pipeline.ingest_depth(
                camera_id, capture_ns, make_depth(depth_m, rng, 0.0006),
                scale_m=0.001)
            pipeline.ingest_pose(camera_id, capture_ns, camera_transform)
            pipeline.ingest_detections(
                camera_id, 999, capture_ns, capture_ns + latency_ns,
                [detection_at(*pixel)])
        targets = pipeline.flush(capture_ns + flush_delay_ns)
        if targets:
            target = targets[0]
            recheck = {
                'phase': target.phase.name,
                'recheck_required': bool(target.recheck_required),
                'weights': {key: round(value, 4)
                            for key, value in target.weights.items()},
            }

    return {
        'frames': frames,
        'target_presence_rate': targets_present / max(1, frames),
        'fused': stats(fused_errors),
        'external_only': stats(external_errors),
        'wrist_only': stats(wrist_errors),
        'occlusion_window': stats(occlusion_errors),
        'phases': phases,
        'drop_recheck': recheck,
        'latency_ms': gpu_latency_ms,
    }


def main(argv=None):
    """Parse arguments, run and print the metrics."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--gpu-latency-ms', type=float, default=30.0)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)
    metrics = run(args.duration, args.seed, args.gpu_latency_ms)
    if args.json:
        print(json.dumps(metrics, indent=2, sort_keys=True))
    else:
        print('frames: {}  target presence: {:.1%}'.format(
            metrics['frames'], metrics['target_presence_rate']))
        for key in ('external_only', 'wrist_only', 'fused',
                    'occlusion_window'):
            value = metrics[key]
            if value.get('n'):
                print(
                    '{:18s} n={:3d} rmse={:.2f} mm p95={:.2f} mm '
                    'max={:.2f} mm'.format(
                        key, value['n'], value['rmse_m'] * 1000.0,
                        value['p95_m'] * 1000.0,
                        value['max_m'] * 1000.0))
        print('phases:', metrics['phases'])
        print('drop recheck:', metrics['drop_recheck'])
    fused = metrics['fused']
    occlusion = metrics['occlusion_window']
    checks = [
        fused.get('rmse_m', 1.0) < 0.006,
        fused.get('max_m', 1.0) < 0.020,
        occlusion.get('max_m', 1.0) < 0.020,
        metrics['target_presence_rate'] > 0.95,
        metrics['drop_recheck'] is not None
        and metrics['drop_recheck']['weights'].get(
            'aurora_wrist', 0.0) == 0.0,
    ]
    if not all(checks):
        print('SIMULATION CHECK FAILED: {}'.format(checks), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
