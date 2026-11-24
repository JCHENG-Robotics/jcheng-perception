#!/usr/bin/env python3
"""Constrain the wrist hand-eye from repeated fixed-point CSVs.

The wrist camera sees the same physical point from several arm poses.  With
``p_i`` the point in ``base_link``, ``q_i`` the tag vector in the camera and
``T0`` the deployed hand-eye (``link4 <- camera``), the corrected hand-eye is
``T_corr = T0 @ dT`` where ``dT`` is a small rigid correction in the camera
frame.  The physical point is fixed, so

    T_bl4_i @ T_corr @ q_i        is the same for every pose i

and ``dT`` is found by minimising the scatter of that point.  A translation
error ``delta`` in the camera frame moves the base point by ``R_bl4 R0 delta``;
a rotation error ``epsilon`` moves it by about ``|epsilon| * |q|`` (5.2 mm per
degree at 0.30 m).  Poses that differ only in joint1 leave the rotation error
about the vertical unobservable, which is why run 3 could not test it.

The reported 1-sigma values are the formal covariance at the given per-pose
noise (default: the fit residual, floored at 0.3 mm), so the output says both
what the data supports and what the pose set can resolve.

Example:

    python3 scripts/handeye_translation_check.py \
        --csv evidence/hardware_20261124_fixedpoint/fixed_point_wrist_...csv \
        --rotation-link4-camera config/wrist_tf.yaml
"""

import argparse
import csv
import json
import math
import sys

import numpy as np

POINT_COLUMNS = {
    'depth': ('p_base_x', 'p_base_y', 'p_base_z'),
    'pnp': ('pnp_base_x', 'pnp_base_y', 'pnp_base_z'),
}
CAMERA_COLUMNS = {
    'depth': ('p_cam_x', 'p_cam_y', 'p_cam_z'),
    'pnp': ('pnp_cam_x', 'pnp_cam_y', 'pnp_cam_z'),
}
ROTATION_COLUMNS = [
    'rot_r00', 'rot_r01', 'rot_r02',
    'rot_r10', 'rot_r11', 'rot_r12',
    'rot_r20', 'rot_r21', 'rot_r22',
]


def load_rows(path):
    with open(path, newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def rodrigues(vector):
    """Rotation matrix from an axis-angle vector."""
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    cross = np.array([[0.0, -axis[2], axis[1]],
                      [axis[2], 0.0, -axis[0]],
                      [-axis[1], axis[0], 0.0]])
    return (np.eye(3) + math.sin(angle) * cross
            + (1.0 - math.cos(angle)) * (cross @ cross))


def load_link4_camera(path):
    """Read ``link4_camera`` (4x4 row-major) from wrist_tf.yaml."""
    import yaml
    with open(path, encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    matrix = np.asarray(config['link4_camera'], dtype=np.float64)
    if matrix.shape != (4, 4):
        raise SystemExit('link4_camera must be 4x4 in {}'.format(path))
    return matrix


def rotation_angle_deg(r_a, r_b):
    cosine = (np.trace(r_a.T @ r_b) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def group_poses(rows, point_columns, camera_columns, min_samples,
                max_spread_m):
    """Group rows by pose_seq; drop travel and thin groups."""
    groups = {}
    for row in rows:
        try:
            point = np.array([float(row[name]) for name in point_columns])
            camera = np.array([float(row[name]) for name in camera_columns])
            rotation = np.array(
                [float(row[name]) for name in ROTATION_COLUMNS]).reshape(3, 3)
            origin = np.array([float(row['cam_' + axis]) for axis in 'xyz'])
        except (KeyError, ValueError):
            return None
        groups.setdefault(int(row['pose_seq']), []).append(
            (point, camera, rotation, origin))
    kept = {}
    dropped = []
    for sequence, samples in sorted(groups.items()):
        origins = np.array([sample[3] for sample in samples])
        spread = float(np.ptp(origins, axis=0).max())
        if len(samples) < min_samples or spread > max_spread_m:
            dropped.append({'pose_seq': sequence, 'n': len(samples),
                            'origin_spread_m': round(spread, 5)})
            continue
        points = np.array([sample[0] for sample in samples])
        cameras = np.array([sample[1] for sample in samples])
        rotations = np.array([sample[2] for sample in samples])
        kept[sequence] = {
            'n': len(samples),
            'point_m': np.median(points, axis=0),
            'point_sd_mm': float(np.sqrt(np.mean(np.var(points, axis=0)))),
            'camera_m': np.median(cameras, axis=0),
            'rotation': rotations.mean(axis=0),
            'origin_m': np.median(origins, axis=0),
            'origin_spread_m': spread,
        }
    return kept, dropped


def link4_pose(pose, hand_eye):
    """Reconstruct T_base_link4 from the CSV camera origin and rotation."""
    rotation = pose['rotation'] @ hand_eye[:3, :3].T
    translation = pose['origin_m'] - rotation @ hand_eye[:3, 3]
    return rotation, translation


def corrected_points(poses, hand_eye, correction):
    """Base-frame tag points for a correction (rotation, translation)."""
    rotation_error, translation_error = correction
    rotation_delta = rodrigues(rotation_error)
    points = []
    for pose in poses:
        rotation, translation = link4_pose(pose, hand_eye)
        camera_point = (
            hand_eye[:3, :3] @ (rotation_delta @ pose['camera_m']
                                + translation_error)
            + hand_eye[:3, 3])
        points.append(translation + rotation @ camera_point)
    return np.array(points)


def residual_vector(theta, poses, hand_eye):
    points = corrected_points(poses, hand_eye, (theta[:3], theta[3:]))
    return (points - points.mean(axis=0)).reshape(-1)


def fit_hand_eye(poses, hand_eye, noise_mm=None):
    """Fit a 6-DoF correction that makes the base point pose-independent."""
    from scipy.optimize import least_squares

    solution = least_squares(
        residual_vector, np.zeros(6), args=(poses, hand_eye),
        method='lm', xtol=1e-12, ftol=1e-12, gtol=1e-12)
    theta = solution.x
    rotation_error = theta[:3]
    translation_error = theta[3:]

    points = corrected_points(
        poses, hand_eye, (rotation_error, translation_error))
    residual_mm = float(
        np.sqrt(np.mean((points - points.mean(axis=0)) ** 2)) * 1000.0)
    raw_points = np.array([pose['point_m'] for pose in poses])
    raw_mm = float(
        np.sqrt(np.mean((raw_points - raw_points.mean(axis=0)) ** 2)) * 1000.0)

    jacobian = solution.jac
    singular = np.linalg.svd(jacobian, compute_uv=False)
    rank = int(np.sum(singular > singular[0] * 1e-8)) if len(singular) else 0
    scale = noise_mm if noise_mm is not None else max(residual_mm, 0.3)
    if rank == 6:
        covariance = np.linalg.pinv(jacobian.T @ jacobian) \
            * (scale * 1e-3) ** 2
        sigma = np.sqrt(np.diag(covariance))
    else:
        sigma = np.full(6, np.nan)

    rotation_delta = np.eye(4)
    rotation_delta[:3, :3] = rodrigues(rotation_error)
    rotation_delta[:3, 3] = translation_error
    corrected = hand_eye @ rotation_delta

    return {
        'poses': len(poses),
        'translation_error_mm': (translation_error * 1000.0).tolist(),
        'rotation_error_deg': float(np.degrees(
            np.linalg.norm(rotation_error))),
        'rotation_error_axis': (
            rotation_error / np.linalg.norm(rotation_error)).tolist()
        if np.linalg.norm(rotation_error) > 1e-12 else [0.0, 0.0, 0.0],
        'sigma_translation_mm': (sigma[3:6] * 1000.0).tolist(),
        'sigma_rotation_deg': np.degrees(sigma[:3]).tolist(),
        'noise_assumed_mm': float(scale),
        'residual_rms_mm': residual_mm,
        'no_correction_residual_rms_mm': raw_mm,
        'rank': rank,
        'jvp_singular_values': singular.tolist(),
        'T_link4_camera_corrected': corrected.tolist(),
        'translation_corrected_m': corrected[:3, 3].tolist(),
        'pose_points_mm': (raw_points * 1000.0).tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', action='append', required=True)
    parser.add_argument('--rotation-link4-camera', required=True)
    parser.add_argument('--min-samples', type=int, default=8)
    parser.add_argument('--max-spread-mm', type=float, default=2.0)
    parser.add_argument('--noise-mm', type=float, default=None,
                        help='per-pose point noise for sigma (default: '
                             'fit residual, floor 0.3 mm)')
    parser.add_argument('--out')
    args = parser.parse_args()

    hand_eye = load_link4_camera(args.rotation_link4_camera)
    report = {'hand_eye_source': args.rotation_link4_camera,
              'hand_eye': hand_eye.tolist()}
    for path in args.csv:
        rows = load_rows(path)
        report[path] = {}
        for name in POINT_COLUMNS:
            if rows and POINT_COLUMNS[name][0] not in rows[0]:
                report[path][name] = {'skipped': 'point columns missing'}
                continue
            grouped = group_poses(
                rows, POINT_COLUMNS[name], CAMERA_COLUMNS[name],
                args.min_samples, args.max_spread_mm * 1e-3)
            if grouped is None:
                report[path][name] = {
                    'skipped': 'rotation columns missing (pre-2026-11-24 '
                               'CSV)'}
                continue
            poses, dropped = grouped
            if len(poses) < 4:
                report[path][name] = {
                    'skipped': 'fewer than 4 stable poses',
                    'poses_kept': len(poses), 'poses_dropped': dropped}
                continue
            pose_list = [poses[sequence] for sequence in sorted(poses)]
            result = fit_hand_eye(pose_list, hand_eye, args.noise_mm)
            result['poses_dropped'] = dropped
            result['pose_sd_mm'] = [pose['point_sd_mm'] for pose in pose_list]
            rotations = [link4_pose(pose, hand_eye)[0]
                         for pose in pose_list]
            result['rotations_deg_from_first'] = [
                rotation_angle_deg(rotations[0], rotation)
                for rotation in rotations]
            report[path][name] = result

    text = json.dumps(report, indent=2)
    print(text)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as handle:
            handle.write(text + '\n')

    worst = 0.0
    for per_file in report.values():
        if not isinstance(per_file, dict):
            continue
        for result in per_file.values():
            if not isinstance(result, dict) or result.get('rank') != 6:
                continue
            delta = np.array(result['translation_error_mm'])
            sigma = np.array(result['sigma_translation_mm'])
            worst = max(worst, float(np.max(np.abs(delta)
                                            / np.maximum(sigma, 1e-9))))
    return 0 if worst <= 3.0 else 1


if __name__ == '__main__':
    sys.exit(main())
