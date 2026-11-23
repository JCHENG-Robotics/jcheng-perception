"""Configuration parsing and validation tests."""

import numpy as np
import pytest

from helpers import REPO_ROOT, load_frames, load_fusion, load_specs
from jcheng_perception_core import Role, load_gpu_config


def test_real_camera_config():
    specs = load_specs()
    assert set(specs) == {'d435i_ext', 'aurora_wrist'}
    assert specs['d435i_ext'].role is Role.EXTERNAL
    assert specs['aurora_wrist'].role is Role.WRIST
    assert specs['d435i_ext'].optical_frame == 'camera_color_optical_frame'
    assert specs['aurora_wrist'].optical_frame == \
        'camera_optical_calibrated'
    assert specs['d435i_ext'].intrinsics.shape == (3, 3)
    assert specs['aurora_wrist'].distortion.shape == (5,)
    assert specs['d435i_ext'].max_uplink_fps == 15.0
    assert specs['aurora_wrist'].depth_match_tolerance_ms == 150.0


def test_fusion_config_priors():
    fusion = load_fusion()
    assert fusion.target_frame == 'base_link'
    assert fusion.prior('FINE', Role.WRIST) > fusion.prior(
        'FINE', Role.EXTERNAL)
    assert fusion.prior('DROP_RECHECK', Role.WRIST) == 0.0
    assert fusion.confirm_hits == 3
    with pytest.raises(ValueError):
        fusion.prior('NOT_A_PHASE', Role.WRIST)  # type: ignore[arg-type]


def test_static_frame_is_orthonormal():
    frames = load_frames()
    transform = frames['d435i_ext']
    rotation = transform[:3, :3]
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(rotation) - 1.0) < 1e-9
    assert np.allclose(
        transform[:3, 3], [0.1393005, 0.0270410, 0.0367494], atol=1e-6)


def test_gpu_config_real_and_mock():
    real = load_gpu_config(str(REPO_ROOT / 'config' / 'gpu.yaml'))
    assert real['cameras']['d435i_ext']['model']['backend'] == 'ultralytics'
    assert real['cameras']['d435i_ext']['model']['path'].endswith('.pt')
    assert real['cameras']['aurora_wrist']['model']['task'] == 'pose'
    mock = load_gpu_config(str(REPO_ROOT / 'config' / 'gpu_mock.yaml'))
    assert mock['cameras']['d435i_ext']['model']['backend'] == 'mock'
    assert mock['node']['debug_publish'] is False


def test_shadow_config_keeps_outputs_off_production_topics():
    shadow = load_gpu_config(
        str(REPO_ROOT / 'config' / 'gpu_shadow.example.yaml'))
    for camera_id, entry in shadow['cameras'].items():
        assert entry['detections_topic'].startswith(
            '/perception/detections_shadow/')
        assert entry['debug_topic'].startswith('/perception/debug_shadow/')
        assert camera_id in ('d435i_ext', 'aurora_wrist')


def test_synthetic_camera_config_loads():
    from jcheng_perception_core import load_camera_specs

    specs = load_camera_specs(
        str(REPO_ROOT / 'config' / 'cameras_synthetic.yaml'))
    assert specs['d435i_ext'].optical_frame == 'synthetic_ext_optical'
    assert specs['aurora_wrist'].optical_frame == 'synthetic_wrist_optical'


def test_invalid_camera_config_raises(tmp_path):
    from jcheng_perception_core import load_camera_specs

    path = tmp_path / 'bad.yaml'
    path.write_text(
        'cameras:\n'
        '  only_one:\n'
        '    role: external\n'
        '    rgb_topic: /image\n'
        '    camera_info_topic: /info\n'
        '    optical_frame: cam\n',
        encoding='utf-8')
    with pytest.raises(ValueError):
        load_camera_specs(str(path))


def test_invalid_fusion_timing_raises(tmp_path):
    from jcheng_perception_core import load_fusion_config

    path = tmp_path / 'bad_fusion.yaml'
    path.write_text(
        'coast_after_s: 2.0\nlost_after_s: 1.0\n', encoding='utf-8')
    with pytest.raises(ValueError):
        load_fusion_config(str(path))
