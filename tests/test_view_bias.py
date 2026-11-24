"""Per-view bias model, view-consistent velocity and drift regression tests."""

import numpy as np
import pytest

from helpers import REPO_ROOT, load_fusion, load_specs
from jcheng_perception_core import (
    Phase,
    TargetObservation,
    TrackManager,
    TrackStatus,
    ViewBiasModel,
    offset_statistics,
    paired_offsets,
)
from jcheng_perception_core.config import FusionConfig


def _observation(camera_id, position, stamp_ns, covariance=1e-6):
    """Build a minimal observation for tracker-level tests."""
    specs = load_specs()
    return TargetObservation(
        camera_id=camera_id,
        role=specs[camera_id].role,
        label='cup',
        class_id=0,
        capture_ns=int(stamp_ns),
        received_ns=int(stamp_ns),
        frame_seq=0,
        frame_id_camera='',
        detection_score=0.9,
        keypoint_score=0.0,
        position_base=np.asarray(position, dtype=np.float64),
        covariance_base=np.eye(3) * float(covariance),
        depth_m=0.30,
        valid_depth_fraction=1.0,
        occlusion_ratio=0.0,
    )


def _manager(**overrides):
    """Build a TrackManager with a deterministic fusion policy."""
    fusion = FusionConfig(**{
        'coalesce_ms': 600.0,
        'publish_rate_hz': 30.0,
        'initial_velocity_sigma_mps': 0.25,
        'max_extrapolation_s': 0.35,
        'enable_view_offset': True,
        'view_offset_alpha': 0.35,
        'view_offset_pair_max_age_s': 1.0,
        'view_offset_sigma_m': 0.020,
        'view_offset_floor_sigma_m': 0.004,
        **overrides,
    })
    return TrackManager(load_specs(), fusion)


def _alternating_run(manager, truth, offset, start_ns=1_000_000_000,
                     period_ns=333_000_000, count=36):
    """Feed external and wrist observations alternately and publish at 30 Hz."""
    targets = []
    track = None
    for index in range(count):
        stamp_ns = start_ns + index * period_ns
        camera_id = 'd435i_ext' if index % 2 == 0 else 'aurora_wrist'
        position = truth + offset if camera_id == 'd435i_ext' else truth
        observation = _observation(camera_id, position, stamp_ns)
        assignments = manager.associate([observation], stamp_ns)
        track = assignments[0][0]
        manager.update(assignments, Phase.COARSE, stamp_ns)
        manager.expire(stamp_ns)
        # Publish a few times between detections, like the 30 Hz edge timer.
        for tick in range(1, 4):
            now_ns = stamp_ns + int(tick * period_ns / 4)
            for target in manager.targets(now_ns, Phase.COARSE):
                targets.append((now_ns, np.asarray(target.position_base)))
    return track, targets


def test_view_bias_model_applies_offset_and_inflates_covariance():
    model = ViewBiasModel.from_config(
        str(REPO_ROOT / 'config' / 'view_bias.yaml'))
    raw = _observation('d435i_ext', [0.4, 0.0, -0.08], 1)
    corrected = model.correct(raw)
    assert np.allclose(
        corrected.position_base - raw.position_base,
        [0.0099, -0.0018, 0.0094], atol=1e-9)
    expected = np.diag([0.006 ** 2, 0.006 ** 2, 0.006 ** 2])
    assert np.allclose(
        corrected.covariance_base - raw.covariance_base, expected)
    # The reference view and unknown cameras are untouched.
    wrist = _observation('aurora_wrist', [0.4, 0.0, -0.08], 1)
    assert model.correct(wrist) is wrist
    assert np.allclose(model.offset('unknown_camera'), 0.0)


def test_view_bias_model_rejects_negative_sigma(tmp_path):
    path = tmp_path / 'bias.yaml'
    path.write_text(
        'views:\n'
        '  d435i_ext:\n'
        '    offset_m: [0.0, 0.0, 0.0]\n'
        '    sigma_m: [0.01, -0.01, 0.0]\n',
        encoding='utf-8')
    with pytest.raises(ValueError):
        ViewBiasModel.from_config(str(path))


def test_paired_offsets_and_statistics_recover_the_camera_offset():
    truth = np.array([0.40, 0.02, -0.08])
    external_offset = np.array([0.010, -0.002, 0.009])
    observations = []
    for index in range(4):
        stamp = 1_000_000_000 + index * 200_000_000
        observations.append(_observation('d435i_ext', truth + external_offset, stamp))
        observations.append(_observation('aurora_wrist', truth, stamp + 1_000_000))
    residuals = paired_offsets(observations, 'aurora_wrist')
    stats = offset_statistics(residuals)
    assert stats['d435i_ext']['count'] == 4
    # Correction is the negative of the measured external-minus-reference.
    assert np.allclose(stats['d435i_ext']['mean'], -external_offset, atol=1e-9)
    assert np.allclose(stats['d435i_ext']['sigma_mean'], 0.0, atol=1e-9)


def test_track_learns_view_offset_and_does_not_drift():
    truth = np.array([0.40, 0.02, -0.08])
    offset = np.array([0.067, -0.013, 0.025])
    manager = _manager()
    track, targets = _alternating_run(manager, truth, offset)
    assert track is not None
    assert track.view_offset_hits > 3
    assert np.allclose(track.view_offset, offset, atol=0.004)
    # The published position must settle on the reference view, not wander.
    settled = np.asarray([position for _, position in targets[60:]])
    assert np.max(np.linalg.norm(settled - truth, axis=1)) < 0.008
    assert np.max(np.linalg.norm(track.kf.velocity)) < 0.02


def test_cross_view_bias_never_becomes_velocity():
    truth = np.array([0.40, 0.02, -0.08])
    offset = np.array([0.067, -0.013, 0.025])
    manager = _manager(enable_view_offset=False)
    track, targets = _alternating_run(manager, truth, offset)
    assert track is not None
    # Without the offset model the position alternates between the views ...
    settled = np.asarray([position for _, position in targets[60:]])
    assert np.max(np.linalg.norm(settled - truth, axis=1)) > 0.04
    # ... but the alternating views must never turn into velocity.
    assert np.max(np.linalg.norm(track.kf.velocity)) < 0.02


def test_reference_point_keeps_the_first_pair_step_small():
    """A defined point (tag/keypoint) makes the first-pair step millimetres."""
    truth = np.array([0.30, 0.05, -0.085])
    residual = np.array([0.003, -0.002, 0.001])
    manager = _manager()
    track, targets = _alternating_run(manager, truth, residual)
    assert track is not None
    assert track.view_offset_hits > 3
    assert np.allclose(track.view_offset, residual, atol=0.002)
    positions = np.asarray([position for _, position in targets])
    assert np.max(np.linalg.norm(positions - truth, axis=1)) < 0.005


def test_velocity_requires_a_minimum_capture_baseline():
    """Near-duplicate capture stamps must not fabricate velocity."""
    truth = np.array([0.30, 0.05, -0.085])
    manager = _manager()
    track = manager._create_track(_observation('d435i_ext', truth, 0))
    track.view_samples['d435i_ext'] = (
        0, truth.copy(), np.eye(3) * 1e-6)
    almost_same = _observation(
        'd435i_ext', truth + np.array([0.001, 0.0, 0.0]), 40_000_000)
    assert manager._same_view_velocity(track, [almost_same]) is None
    normal = _observation(
        'd435i_ext', truth + np.array([0.003, 0.0, 0.0]), 300_000_000)
    measurement = manager._same_view_velocity(track, [normal])
    assert measurement is not None
    assert measurement[0][0] == pytest.approx(0.010, abs=1e-3)


def test_published_extrapolation_is_capped():
    manager = _manager()
    truth = np.array([0.40, 0.02, -0.08])
    track = manager._create_track(_observation('d435i_ext', truth, 0))
    track.status = TrackStatus.CONFIRMED
    track.last_update_ns = 0
    track.last_state_ns = 0
    track.kf.state[3:] = np.array([1.0, 0.0, 0.0])
    targets = manager.targets(2_000_000_000, Phase.COARSE)
    assert len(targets) == 1
    travel = targets[0].position_base[0] - track.kf.position[0]
    assert travel == pytest.approx(0.35, abs=1e-6)


def test_pipeline_applies_the_chain_correction():
    from jcheng_perception_core import PerceptionPipeline

    model = ViewBiasModel.from_config(
        str(REPO_ROOT / 'config' / 'view_bias.yaml'))
    from helpers import feed_frame, make_detection
    from jcheng_perception_core.geometry import make_transform

    pipeline = PerceptionPipeline(
        load_specs(), load_fusion('fusion.yaml'), {}, model)
    stamp = 1_000_000_000
    detection = feed_frame(pipeline, 'd435i_ext', stamp, make_transform(),
                           1.0, 1, detection=make_detection())
    pipeline.ingest_detections(
        'd435i_ext', 1, stamp, stamp + 1_000_000, [detection], 'test')
    pipeline.flush(stamp + 60_000_000)
    observations = pipeline.take_observations()
    assert len(observations) == 1
    # The raw back-projection is on the optical axis at 1 m; the external
    # correction shifts it by the configured offset.
    assert np.allclose(
        observations[0].position_base, [0.0099, -0.0018, 1.0094], atol=1e-6)
