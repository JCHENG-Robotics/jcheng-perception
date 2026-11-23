"""Association, Kalman, weighting and end-to-end pipeline tests."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from helpers import (
    K,
    depth_array,
    feed_frame,
    load_fusion,
    load_specs,
    make_detection,
    transform,
)
from jcheng_perception_core import (
    ConstantVelocityKalman,
    PerceptionPipeline,
    Phase,
    Role,
    TrackManager,
    TrackStatus,
    fuse_observations,
    phase_for_observations,
)
from jcheng_perception_core.weighting import range_penalty, view_factor


def _pipeline():
    """Create a pipeline with balanced depth noise for association tests."""
    specs = load_specs()
    for spec in specs.values():
        spec.depth_noise_quadratic = 0.0025
        spec.depth_noise_floor_m = 0.0015
    fusion = load_fusion('fusion_demo.yaml')
    return PerceptionPipeline(specs, fusion, {})


def _round(pipeline, camera_id, stamp_ns, pose, depth_m, seq,
           detection=None):
    """Store one frame and one detection batch."""
    detection = feed_frame(
        pipeline, camera_id, stamp_ns, pose, depth_m, seq=seq,
        detection=detection)
    pipeline.ingest_detections(
        camera_id, seq, stamp_ns, stamp_ns + 10_000_000, [detection])


def test_kalman_predict_update_and_gating():
    kf = ConstantVelocityKalman(
        [0.0, 0.0, 0.0], np.eye(3) * 0.01,
        velocity=[1.0, 0.0, 0.0], accel_noise=0.0)
    kf.predict(1.0)
    assert np.allclose(kf.position, [1.0, 0.0, 0.0], atol=1e-9)
    before = np.trace(kf.position_covariance)
    kf.update([1.1, 0.0, 0.0], np.eye(3) * 0.001)
    after = np.trace(kf.position_covariance)
    assert after < before
    assert 1.0 < kf.position[0] < 1.1
    assert math.isfinite(kf.gating_distance(
        [1.1, 0.0, 0.0], np.eye(3) * 0.001))


def test_weights_prefer_wrist_in_fine_phase():
    specs = load_specs()
    fusion = load_fusion()
    from helpers import observation

    external = observation(
        'd435i_ext', Role.EXTERNAL, sigma=0.002, depth_m=0.4)
    wrist = observation(
        'aurora_wrist', Role.WRIST, sigma=0.002, depth_m=0.4)
    fused = fuse_observations(
        [external, wrist], Phase.FINE, specs, fusion)
    assert fused is not None
    assert fused.weights['aurora_wrist'] > 0.8
    assert fused.weights['d435i_ext'] < 0.2
    assert abs(sum(fused.weights.values()) - 1.0) < 1e-12


def test_weights_prefer_external_for_drop_recheck():
    specs = load_specs()
    fusion = load_fusion()
    from helpers import observation

    external = observation('d435i_ext', Role.EXTERNAL, sigma=0.002)
    wrist = observation('aurora_wrist', Role.WRIST, sigma=0.002)
    fused = fuse_observations(
        [external, wrist], Phase.DROP_RECHECK, specs, fusion)
    assert fused is not None
    assert fused.weights == {'d435i_ext': 1.0}
    assert fused.sources == ['d435i_ext']


def test_inverse_variance_fusion_halves_equal_covariance():
    specs = load_specs()
    fusion = load_fusion()
    from helpers import observation

    external = observation('d435i_ext', Role.EXTERNAL, sigma=0.002)
    wrist = observation('aurora_wrist', Role.WRIST, sigma=0.002)
    fused = fuse_observations(
        [external, wrist], Phase.SEARCH, specs, fusion)
    assert fused is not None
    single = np.eye(3) * 0.002 ** 2
    assert np.all(np.diag(fused.covariance_base) <
                  np.diag(single) * 0.6)


def test_range_penalty_and_occlusion_reduce_factor():
    specs = load_specs()
    fusion = load_fusion()
    from helpers import observation

    wrist_spec = specs['aurora_wrist']
    assert range_penalty(0.4, wrist_spec) == 1.0
    assert range_penalty(1.5, wrist_spec) < 0.1
    near = observation(
        'aurora_wrist', Role.WRIST, sigma=0.002, depth_m=0.4)
    far = observation(
        'aurora_wrist', Role.WRIST, sigma=0.002, depth_m=1.5)
    assert view_factor(near, Phase.FINE, wrist_spec, fusion) > \
        view_factor(far, Phase.FINE, wrist_spec, fusion)
    occluded = observation(
        'aurora_wrist', Role.WRIST, sigma=0.002, depth_m=0.4,
        occlusion=1.0)
    assert view_factor(occluded, Phase.FINE, wrist_spec, fusion) < \
        view_factor(near, Phase.FINE, wrist_spec, fusion)


def test_phase_policy():
    fusion = load_fusion()
    wrist_close = SimpleNamespace(
        role=Role.WRIST, depth_m=0.30)
    external = SimpleNamespace(
        role=Role.EXTERNAL, depth_m=1.2)
    assert phase_for_observations([], [], fusion) is Phase.SEARCH
    assert phase_for_observations(
        [external], [], fusion) is Phase.COARSE
    assert phase_for_observations(
        [wrist_close], [SimpleNamespace(wrist_hits=0)],
        fusion) is Phase.ACQUIRE
    assert phase_for_observations(
        [wrist_close], [SimpleNamespace(wrist_hits=3)],
        fusion) is Phase.FINE
    assert phase_for_observations(
        [wrist_close, external], [SimpleNamespace(wrist_hits=3)],
        fusion) is Phase.FINE


def test_pipeline_single_view_tracks_and_confirms():
    pipeline = _pipeline()
    t0 = 1_000_000_000
    _round(pipeline, 'd435i_ext', t0, transform(), 1.0, 1)
    targets = pipeline.flush(t0 + 60_000_000)
    assert len(targets) == 1
    target = targets[0]
    assert np.allclose(target.position_base, [0.0, 0.0, 1.0], atol=1e-6)
    assert target.status is TrackStatus.CONFIRMED
    assert target.phase is Phase.COARSE
    assert target.source_cameras == ['d435i_ext']
    assert target.weights == {'d435i_ext': 1.0}


def test_pipeline_associates_two_views_into_one_track():
    pipeline = _pipeline()
    t0 = 1_000_000_000
    _round(pipeline, 'd435i_ext', t0, transform(), 1.0, 1)
    _round(pipeline, 'aurora_wrist', t0, transform(), 1.0, 1)
    targets = pipeline.flush(t0 + 60_000_000)
    assert len(targets) == 1
    target = targets[0]
    assert set(target.source_cameras) == {'d435i_ext', 'aurora_wrist'}
    assert abs(sum(target.weights.values()) - 1.0) < 1e-9
    assert target.weights['d435i_ext'] > 0.4
    assert target.weights['aurora_wrist'] > 0.4
    assert target.phase is Phase.ACQUIRE
    assert target.depth_by_camera['d435i_ext'] == 1.0


def test_pipeline_matches_older_frame_by_capture_stamp():
    pipeline = _pipeline()
    t0 = 1_000_000_000
    feed_frame(pipeline, 'd435i_ext', t0, transform(), 1.0, seq=1)
    feed_frame(pipeline, 'd435i_ext', t0 + 100_000_000,
               transform(), 2.0, seq=2)
    detection = make_detection()
    pipeline.ingest_detections(
        'd435i_ext', 99, t0 + 40_000_000, t0 + 45_000_000,
        [detection])
    targets = pipeline.flush(t0 + 100_000_000)
    assert len(targets) == 1
    assert abs(targets[0].position_base[2] - 1.0) < 1e-6
    assert targets[0].depth_by_camera['d435i_ext'] == 1.0


def test_pipeline_interpolates_tf_at_depth_stamp():
    pipeline = _pipeline()
    t0 = 1_000_000_000
    feed_frame(pipeline, 'd435i_ext', t0, transform(), 1.0, seq=1)
    pipeline.ingest_pose(
        'd435i_ext', t0 + 100_000_000, transform(x=0.2))
    # No pose sample at 50 ms: the depth frame is matched with an interpolated
    # TF sample, not with the newest pose.
    pipeline.ingest_frame(
        'd435i_ext', 2, t0 + 50_000_000, t0 + 51_000_000,
        640, 400, 'cam', K)
    pipeline.ingest_depth(
        'd435i_ext', t0 + 50_000_000, depth_array(1.0))
    detection = make_detection()
    pipeline.ingest_detections(
        'd435i_ext', 2, t0 + 50_000_000, t0 + 55_000_000,
        [detection])
    targets = pipeline.flush(t0 + 100_000_000)
    assert len(targets) == 1
    assert abs(targets[0].position_base[0] - 0.1) < 1e-3
    assert abs(targets[0].position_base[2] - 1.0) < 1e-6


def test_pipeline_drops_detection_without_depth_or_tf():
    pipeline = _pipeline()
    t0 = 1_000_000_000
    pipeline.ingest_frame(
        'd435i_ext', 1, t0, t0 + 1_000_000, 640, 400, 'cam', K)
    pipeline.ingest_pose('d435i_ext', t0, transform())
    detection = make_detection()
    pipeline.ingest_detections(
        'd435i_ext', 1, t0, t0 + 10_000_000, [detection])
    targets = pipeline.flush(t0 + 60_000_000)
    assert targets == []
    assert pipeline.health['d435i_ext'].depth_match_failures == 1
    assert pipeline.total_dropped == 1

    pipeline = _pipeline()
    pipeline.ingest_frame(
        'd435i_ext', 2, t0, t0 + 1_000_000, 640, 400, 'cam', K)
    pipeline.ingest_depth('d435i_ext', t0, depth_array(1.0))
    pipeline.ingest_detections(
        'd435i_ext', 2, t0, t0 + 10_000_000, [detection])
    targets = pipeline.flush(t0 + 60_000_000)
    assert targets == []
    assert pipeline.health['d435i_ext'].tf_match_failures == 1


def test_pipeline_drop_recheck_override_uses_external_only():
    pipeline = _pipeline()
    t0 = 1_000_000_000
    _round(pipeline, 'd435i_ext', t0, transform(), 1.0, 1)
    _round(pipeline, 'aurora_wrist', t0, transform(), 1.0, 1)
    pipeline.flush(t0 + 60_000_000)
    pipeline.set_phase_override(Phase.DROP_RECHECK, t0 + 60_000_000)
    t1 = t0 + 200_000_000
    _round(pipeline, 'd435i_ext', t1, transform(), 1.0, 2)
    _round(pipeline, 'aurora_wrist', t1, transform(), 1.0, 2)
    targets = pipeline.flush(t1 + 60_000_000)
    assert len(targets) == 1
    target = targets[0]
    assert target.phase is Phase.DROP_RECHECK
    assert target.recheck_required
    assert target.weights == {'d435i_ext': 1.0}
    assert target.weights.get('aurora_wrist', 0.0) == 0.0


def test_pipeline_drops_lost_track():
    pipeline = _pipeline()
    t0 = 1_000_000_000
    _round(pipeline, 'd435i_ext', t0, transform(), 1.0, 1)
    assert len(pipeline.flush(t0 + 60_000_000)) == 1
    targets = pipeline.flush(t0 + 3_000_000_000)
    assert targets == []
    assert pipeline.tracks.tracks == {}


def test_merge_duplicate_cross_camera_tracks():
    from helpers import observation

    specs = load_specs()
    fusion = load_fusion('fusion_demo.yaml')
    manager = TrackManager(specs, fusion)
    external = observation(
        'd435i_ext', Role.EXTERNAL, position=(0.35, 0.02, -0.08),
        sigma=0.002, depth_m=0.4)
    wrist = observation(
        'aurora_wrist', Role.WRIST, position=(0.354, 0.022, -0.08),
        sigma=0.002, depth_m=0.4)
    external_track = manager._create_track(external)
    wrist_track = manager._create_track(wrist)
    manager.apply_group(
        external_track, [external], Phase.COARSE, 1_100_000_000)
    manager.apply_group(
        wrist_track, [wrist], Phase.FINE, 1_100_000_000)
    merged = manager.merge_duplicates(1_100_000_000)
    assert len(manager.tracks) == 1
    assert merged
    surviving = list(manager.tracks.values())[0]
    assert surviving.wrist_hits >= 1
    assert surviving.external_hits >= 1


def test_merge_does_not_collapse_same_camera_tracks():
    from helpers import observation

    specs = load_specs()
    fusion = load_fusion('fusion_demo.yaml')
    manager = TrackManager(specs, fusion)
    first = observation(
        'd435i_ext', Role.EXTERNAL, position=(0.35, 0.02, -0.08),
        sigma=0.002)
    second = observation(
        'd435i_ext', Role.EXTERNAL, position=(0.38, 0.04, -0.08),
        sigma=0.002)
    first_track = manager._create_track(first)
    second_track = manager._create_track(second)
    manager.apply_group(first_track, [first], Phase.COARSE, 1_100_000_000)
    manager.apply_group(second_track, [second], Phase.COARSE, 1_100_000_000)
    merged = manager.merge_duplicates(1_100_000_000)
    assert merged == []
    assert len(manager.tracks) == 2


def test_merge_overlapping_camera_duplicates_inside_strict_gate():
    from helpers import observation

    specs = load_specs()
    fusion = load_fusion('fusion_demo.yaml')
    manager = TrackManager(specs, fusion)
    external = observation(
        'd435i_ext', Role.EXTERNAL, position=(0.35, 0.02, -0.08),
        sigma=0.002)
    wrist = observation(
        'aurora_wrist', Role.WRIST, position=(0.354, 0.022, -0.08),
        sigma=0.002)
    main_track = manager._create_track(external)
    duplicate = manager._create_track(wrist)
    manager.apply_group(main_track, [external], Phase.COARSE, 1_100_000_000)
    manager.apply_group(duplicate, [wrist], Phase.FINE, 1_100_000_000)
    # Pretend both tracks have seen both cameras; the disjoint rule must not
    # leave a 4 mm duplicate alive.
    main_track.last_fused.sources = ['d435i_ext', 'aurora_wrist']
    duplicate.last_fused.sources = ['d435i_ext', 'aurora_wrist']
    merged = manager.merge_duplicates(1_100_000_000)
    assert merged
    assert len(manager.tracks) == 1


def test_edge_and_core_have_no_inference_code():
    root = Path(__file__).resolve().parents[1]
    for package in ('jcheng_perception_core', 'jcheng_perception_edge'):
        for path in (root / 'ros2_ws' / 'src' / package).rglob('*.py'):
            text = path.read_text(encoding='utf-8').lower()
            assert 'from ultralytics' not in text, path
            assert 'import ultralytics' not in text, path
            assert 'yolo(' not in text, path
            assert 'import rclpy' not in text or \
                package == 'jcheng_perception_edge', path
    gpu_detectors = (
        root / 'ros2_ws' / 'src' / 'jcheng_perception_gpu' /
        'jcheng_perception_gpu' / 'detectors.py')
    assert 'ultralytics' in gpu_detectors.read_text(
        encoding='utf-8').lower()
