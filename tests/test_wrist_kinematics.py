"""Kinematics for the measured wrist-camera TF (ROS-free)."""

import math

import numpy as np
import pytest
import yaml

from helpers import REPO_ROOT
from jcheng_perception_edge.wrist_kinematics import (
    forward_kinematics,
    parse_chain,
    pulses_to_positions,
)

URDF = """<?xml version="1.0"?>
<robot name="test">
  <link name="base_link"/>
  <link name="l1"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="l1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/>
    <child link="tip"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>"""


def test_parse_chain_and_forward_kinematics():
    chain = parse_chain(URDF, 'base_link', 'tip')
    assert [joint['name'] for joint in chain] == ['j1', 'j2']
    result = forward_kinematics(chain, {'j1': 0.0, 'j2': math.pi / 2.0})
    assert np.allclose(result[:3, 3], [0.2, 0.0, 0.1], atol=1e-12)
    # Ry(+90) maps the tip z axis to +x.
    assert np.allclose(result[:3, 2], [1.0, 0.0, 0.0], atol=1e-12)
    rotated = forward_kinematics(
        chain, {'j1': math.pi / 2.0, 'j2': 0.0})
    assert np.allclose(rotated[:3, 3], [0.0, 0.2, 0.1], atol=1e-12)


def test_fixed_joint_and_missing_position():
    urdf = URDF.replace(
        '<joint name="j2" type="revolute">',
        '<joint name="j2" type="fixed">')
    chain = parse_chain(urdf, 'base_link', 'tip')
    result = forward_kinematics(chain, {'j1': 0.0})
    assert np.allclose(result[:3, 3], [0.2, 0.0, 0.1], atol=1e-12)
    with pytest.raises(KeyError):
        forward_kinematics(
            parse_chain(URDF, 'base_link', 'tip'), {'j1': 0.0})


def test_unknown_tip_or_duplicate_child_raises():
    with pytest.raises(ValueError):
        parse_chain(URDF, 'base_link', 'missing')
    duplicate = URDF.replace(
        '<child link="tip"/>', '<child link="l1"/>')
    with pytest.raises(ValueError):
        parse_chain(duplicate, 'base_link', 'tip')


def test_pulses_to_positions_inverts_joint3():
    positions = pulses_to_positions(
        {6: 600, 5: 400, 4: 400},
        {'joint1': 6, 'joint2': 5, 'joint3': 4},
        center_pulse=500,
        radians_per_tick=0.004,
        inverted_joints=['joint3'],
    )
    assert positions['joint1'] == pytest.approx(0.4)
    assert positions['joint2'] == pytest.approx(-0.4)
    assert positions['joint3'] == pytest.approx(0.4)
    with pytest.raises(KeyError):
        pulses_to_positions(
            {6: 600}, {'joint1': 6, 'joint2': 5},
            radians_per_tick=0.004)


def test_wrist_tf_config_is_orthonormal():
    with open(REPO_ROOT / 'config' / 'wrist_tf.yaml',
              encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    assert config['camera_frame'] == 'camera_optical_measured'
    matrix = np.asarray(config['link4_camera'], dtype=float)
    assert matrix.shape == (4, 4)
    assert np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0])
    rotation = matrix[:3, :3]
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-9)
    assert np.allclose(
        matrix[:3, 3],
        [-0.0348444217248839, -0.009063734133950539,
         0.0585736292974927],
        atol=1e-12)
