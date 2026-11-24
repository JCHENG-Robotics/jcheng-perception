"""Measured wrist-camera kinematics (no ROS import).

The vendor controller publishes ``/joint_states`` from its command cache, not
from the servos.  The cache can disagree with the bus positions by hundreds of
pulses while the arm is stationary, which makes a TF-derived camera pose wrong.
These helpers rebuild ``base_link -> link4`` from the live URDF and the
positions read through ``/ros_robot_controller/bus_servo/get_state``.

All transforms are 4x4 homogeneous matrices mapping a child frame into its
parent, matching :mod:`jcheng_perception_core.geometry`.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_RADIANS_PER_TICK = 240.0 / 360.0 * 2.0 * math.pi / 1000.0
MAX_CHAIN_LENGTH = 64


def rpy_matrix(roll, pitch, yaw):
    """Return the rotation matrix for fixed-axis roll/pitch/yaw."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_matrix(axis, angle):
    """Return the rotation matrix for a rotation about an arbitrary axis."""
    vector = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError('joint axis must be non-zero')
    x, y, z = vector / norm
    skew = np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])
    return (
        np.eye(3)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


def _joint_spec(joint):
    """Return the origin, axis and type of a URDF joint element."""
    origin = joint.find('origin')
    if origin is None:
        translation = np.zeros(3)
        rotation = np.eye(3)
    else:
        translation = np.array(
            [float(value) for value in origin.get('xyz', '0 0 0').split()],
            dtype=np.float64)
        rpy = [float(value) for value in
               origin.get('rpy', '0 0 0').split()]
        rotation = rpy_matrix(*rpy)
    axis_element = joint.find('axis')
    if axis_element is None:
        axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = np.array(
            [float(value) for value in
             axis_element.get('xyz', '1 0 0').split()],
            dtype=np.float64)
    joint_type = joint.get('type', '')
    if joint_type not in ('revolute', 'continuous', 'fixed'):
        raise ValueError(
            'unsupported joint type {!r} for {!r}'.format(
                joint_type, joint.get('name')))
    return {
        'name': joint.get('name', ''),
        'type': joint_type,
        'translation': translation,
        'rotation': rotation,
        'axis': axis,
    }


def parse_chain(urdf_xml, base_frame, tip_frame):
    """Return the URDF joint chain from ``base_frame`` to ``tip_frame``.

    Only revolute, continuous and fixed joints are supported.  The result is
    ordered from the base towards the tip.
    """
    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError as error:
        raise ValueError('invalid URDF XML: {}'.format(error)) from error
    joint_for_child = {}
    for joint in root.findall('joint'):
        parent_element = joint.find('parent')
        child_element = joint.find('child')
        if parent_element is None or child_element is None:
            raise ValueError(
                'joint {!r} has no parent or child'.format(
                    joint.get('name')))
        child = child_element.get('link')
        if child in joint_for_child:
            raise ValueError(
                'multiple joints declare child link {!r}'.format(child))
        joint_for_child[child] = joint
    chain = []
    link = tip_frame
    while link != base_frame:
        if link not in joint_for_child:
            raise ValueError(
                'no joint chain from {!r} to {!r}'.format(
                    base_frame, tip_frame))
        joint = joint_for_child[link]
        chain.append(_joint_spec(joint))
        link = joint.find('parent').get('link')
        if len(chain) > MAX_CHAIN_LENGTH:
            raise ValueError('joint chain contains a cycle')
    chain.reverse()
    return chain


def forward_kinematics(chain, positions):
    """Return ``T_base_tip`` for a parsed chain and joint positions.

    ``positions`` maps joint name to radians; fixed joints are ignored.
    """
    matrix = np.eye(4)
    for joint in chain:
        origin = np.eye(4)
        origin[:3, :3] = joint['rotation']
        origin[:3, 3] = joint['translation']
        matrix = matrix @ origin
        if joint['type'] == 'fixed':
            continue
        name = joint['name']
        if name not in positions:
            raise KeyError('missing position for joint {!r}'.format(name))
        motion = np.eye(4)
        motion[:3, :3] = axis_angle_matrix(
            joint['axis'], float(positions[name]))
        matrix = matrix @ motion
    return matrix


def pulses_to_positions(pulses, servo_ids, center_pulse=500,
                        radians_per_tick=DEFAULT_RADIANS_PER_TICK,
                        inverted_joints=()):
    """Convert bus-servo pulses to joint radians.

    ``servo_ids`` maps joint name to servo id.  ``inverted_joints`` lists the
    joints whose vendor mapping is ``(center - pulse)`` rather than
    ``(pulse - center)`` (joint3 on the ArmPi Ultra).
    """
    inverted = set(inverted_joints)
    positions = {}
    for joint_name, servo_id in servo_ids.items():
        if servo_id not in pulses:
            raise KeyError('missing servo pulse for id {}'.format(servo_id))
        angle = (float(pulses[servo_id]) - center_pulse) * radians_per_tick
        if joint_name in inverted:
            angle = -angle
        positions[joint_name] = angle
    return positions
