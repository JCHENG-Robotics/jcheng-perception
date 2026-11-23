"""
Rigid-body geometry for the perception pipeline (no ROS dependency).

All transforms are 4x4 homogeneous matrices mapping a point expressed in a
child frame into its parent frame::

    p_parent = T_parent_child @ p_child

Quaternions use the ROS/OpenCV order ``(x, y, z, w)``. Camera optical frames
follow the OpenCV convention: x right, y down, z forward.
"""

import math

import numpy as np

EPS = 1e-12


def normalize_quaternion(quaternion):
    """Return a unit quaternion, raising on a zero-length input."""
    values = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if norm < EPS:
        raise ValueError('cannot normalize a zero quaternion')
    return values / norm


def quaternion_multiply(left, right):
    """Hamilton product of two ``(x, y, z, w)`` quaternions."""
    x1, y1, z1, w1 = np.asarray(left, dtype=np.float64)
    x2, y2, z2, w2 = np.asarray(right, dtype=np.float64)
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quaternion_conjugate(quaternion):
    """Return the inverse of a unit quaternion."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    return np.array([-x, -y, -z, w])


def quaternion_slerp(start, end, fraction):
    """Spherical linear interpolation between two unit quaternions."""
    start = normalize_quaternion(start)
    end = normalize_quaternion(end)
    dot = float(np.dot(start, end))
    if dot < 0.0:
        end = -end
        dot = -dot
    if dot > 0.9995:
        return normalize_quaternion(start + fraction * (end - start))
    theta = math.acos(min(1.0, max(-1.0, dot)))
    sin_theta = math.sin(theta)
    weight_start = math.sin((1.0 - fraction) * theta) / sin_theta
    weight_end = math.sin(fraction * theta) / sin_theta
    return normalize_quaternion(weight_start * start + weight_end * end)


def matrix_from_quaternion(quaternion):
    """Return the 3x3 rotation matrix of a unit quaternion."""
    x, y, z, w = normalize_quaternion(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


def quaternion_from_matrix(rotation):
    """Return the unit quaternion of a 3x3 rotation matrix (Shepperd)."""
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
        ) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
        ) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
        ) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    return normalize_quaternion([x, y, z, w])


def yaw_quaternion(yaw_rad):
    """Return a quaternion for a rotation about +z."""
    half = 0.5 * float(yaw_rad)
    return np.array([0.0, 0.0, math.sin(half), math.cos(half)])


def quaternion_yaw(quaternion):
    """Return the yaw angle of a quaternion, ignoring roll and pitch."""
    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def make_transform(rotation=None, translation=None):
    """Build a 4x4 transform from a 3x3 rotation and a 3-vector."""
    transform = np.eye(4, dtype=np.float64)
    if rotation is not None:
        transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    if translation is not None:
        transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def invert_transform(transform):
    """Invert a rigid 4x4 transform."""
    matrix = np.asarray(transform, dtype=np.float64)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def transform_points(transform, points):
    """Apply a 4x4 transform to an ``(N, 3)`` array."""
    values = np.asarray(points, dtype=np.float64)
    if values.ndim == 1:
        values = values.reshape(1, 3)
    return values @ np.asarray(transform, dtype=np.float64)[:3, :3].T + \
        np.asarray(transform, dtype=np.float64)[:3, 3]


def transform_point(transform, point):
    """Apply a 4x4 transform to a single 3-vector."""
    return transform_points(transform, point)[0]


def compose(left, right):
    """Return ``left @ right`` for two 4x4 transforms."""
    return np.asarray(left, dtype=np.float64) @ np.asarray(
        right, dtype=np.float64)


def rotation_angle_rad(rotation):
    """Return the rotation angle of a 3x3 matrix in radians."""
    matrix = np.asarray(rotation, dtype=np.float64)
    cosine = (float(np.trace(matrix)) - 1.0) / 2.0
    return math.acos(min(1.0, max(-1.0, cosine)))


def deproject_pixel(u, v, depth_m, camera_matrix):
    """
    Back-project a pixel to the camera optical frame.

    ``camera_matrix`` is the 3x3 pinhole intrinsic matrix. Depth is the
    z-coordinate in metres. The returned point is ``(x, y, z)`` in the camera
    optical frame.
    """
    intrinsics = np.asarray(camera_matrix, dtype=np.float64)
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('camera_matrix focal lengths must be positive')
    z = float(depth_m)
    if z <= 0.0:
        raise ValueError('depth must be positive')
    return np.array([
        (float(u) - cx) * z / fx,
        (float(v) - cy) * z / fy,
        z,
    ])


def project_point(point_camera, camera_matrix):
    """Project a camera-frame point to pixel coordinates."""
    intrinsics = np.asarray(camera_matrix, dtype=np.float64)
    x, y, z = np.asarray(point_camera, dtype=np.float64)
    if abs(z) < EPS:
        raise ValueError('cannot project a point on the camera plane')
    return np.array([
        intrinsics[0, 0] * x / z + intrinsics[0, 2],
        intrinsics[1, 1] * y / z + intrinsics[1, 2],
    ])


def _canonical_axis_yaw(major_axis):
    """Return a yaw in [-pi/2, pi/2] for a bidirectional major axis."""
    x = float(major_axis[0])
    y = float(major_axis[1])
    if x < 0.0 or (abs(x) < 1e-12 and y < 0.0):
        x, y = -x, -y
    return math.atan2(y, x)


def plane_pose_from_points(points, max_tilt_deg=30.0, inlier_m=0.012):
    """
    Estimate a table-parallel yaw from base-frame points.

    The normal is fitted with SVD, the inliers are projected onto the fitted
    plane and the major horizontal axis gives the object yaw. A circular object
    has no stable yaw; callers must treat the returned confidence accordingly.

    Returns ``(orientation_xyzw, normal, inlier_ratio)`` or ``None`` when the
    cloud is too small or the plane is not table-like.
    """
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] < 20 or values.shape[1] != 3:
        return None
    finite = np.all(np.isfinite(values), axis=1)
    values = values[finite]
    if values.shape[0] < 20:
        return None

    centroid = values.mean(axis=0)
    centered = values - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    distances = np.abs(centered @ normal)
    inliers = values[distances <= inlier_m]
    if inliers.shape[0] < 15:
        return None

    centroid = inliers.mean(axis=0)
    centered = inliers - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal = -normal
    tilt = math.acos(min(1.0, max(-1.0, float(normal[2]))))
    if tilt > math.radians(max_tilt_deg):
        return None

    xy = inliers[:, :2] - inliers[:, :2].mean(axis=0)
    if xy.shape[0] < 15:
        return None
    covariance = xy.T @ xy / float(xy.shape[0])
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major = eigenvectors[:, int(np.argmax(eigenvalues))]
    yaw = _canonical_axis_yaw(major)
    inlier_ratio = float(inliers.shape[0]) / float(values.shape[0])
    return yaw_quaternion(yaw), normal, inlier_ratio


def object_yaw_from_points(points, table_clearance_m=0.008,
                           max_points=4000, max_tilt_deg=35.0):
    """
    Estimate an object yaw after removing the table plane.

    Points above the fitted table by ``table_clearance_m`` are kept; their
    horizontal PCA major axis defines the yaw. Returns
    ``(orientation_xyzw, height_m, support_ratio)`` or ``None``.
    """
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] < 40:
        return None
    finite = np.all(np.isfinite(values), axis=1)
    values = values[finite]
    if values.shape[0] < 40:
        return None
    if values.shape[0] > max_points:
        stride = int(math.ceil(values.shape[0] / float(max_points)))
        values = values[::stride]

    centroid = values.mean(axis=0)
    centered = values - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal = -normal
    if normal[2] < math.cos(math.radians(max_tilt_deg)):
        return None
    offset = float(centroid @ normal)
    heights = values @ normal - offset
    above = values[heights > float(table_clearance_m)]
    support_ratio = float(above.shape[0]) / float(values.shape[0])
    if above.shape[0] < 15:
        return None
    height = float(np.max(heights) - min(0.0, float(np.min(heights))))
    xy = above[:, :2] - above[:, :2].mean(axis=0)
    covariance = xy.T @ xy / float(xy.shape[0])
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major = eigenvectors[:, int(np.argmax(eigenvalues))]
    if float(np.max(eigenvalues)) < 1e-8:
        return None
    yaw = _canonical_axis_yaw(major)
    return yaw_quaternion(yaw), height, support_ratio
