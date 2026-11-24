#!/usr/bin/env python3
"""Measure one fixed AprilTag target in ``base_link`` from one camera.

The target is the centre of a printed AprilTag 36h11 marker (default ID 0,
60 mm black square, the sticker used by ``jcheng-manipulation``). Every sample
yields two independent estimates of that same physical point:

``depth``  the production path: detected marker-centre pixel -> median depth
           patch -> back-projection with the RGB ``CameraInfo`` -> the live
           ``base_link`` TF (the same steps as
           ``jcheng_perception_core.backproject``, including its preference for
           the message intrinsics);
``pnp``    RGB-only ``solvePnP(IPPE_SQUARE)`` marker pose -> marker centre in
           camera coordinates, independent of depth.

The camera chain is validated by repeating the measurement with the camera in
several poses (wrist) or by comparing two cameras (external D435i vs wrist).
That is what separates a wrong hand-eye/odometry from a depth-registration
effect: ``pnp`` is depth-free, ``depth`` is what fusion actually uses.

Inputs are matched like the production pipeline: the nearest depth frame within
``--depth-tolerance-ms`` of the colour stamp supplies the metres, and the TF is
looked up at the *depth* stamp (falling back to the latest sample, recorded as
``tf_mode=latest``).

The run is paced by a human: hold the camera still at each pose for >= 10 s,
then move to the next pose. Camera-origin jumps > 20 mm start a new
``pose_seq``; the CSV keeps every sample so the analysis can drop travel.

Example (inside the robot container):

    ROS_LOG_DIR=/tmp/roslog python3 fixed_point_check.py --camera wrist \
        --duration 180 --out /tmp/fixed_point_wrist.csv

    python3 fixed_point_check.py --self-test
"""

import argparse
import math
import signal
import sys
import time

import cv2
import numpy as np

try:  # the self-test runs without ROS installed
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener
    ROS_IMPORT_ERROR = None
except ImportError as ROS_IMPORT_ERROR:  # noqa: N816 - module-level flag
    Node = object

CAMERAS = {
    'wrist': {
        'rgb_topic': '/depth_cam/rgb/image_raw',
        'depth_topic': '/depth_cam/depth/image_raw',
        'info_topic': '/depth_cam/rgb/camera_info',
        'camera_frame': 'camera_optical_measured',
        'min_depth_m': 0.08,
        'max_depth_m': 1.20,
    },
    'ext': {
        'rgb_topic': '/camera/color/image_raw',
        'depth_topic': '/camera/aligned_depth_to_color/image_raw',
        'info_topic': '/camera/color/camera_info',
        'camera_frame': 'camera_color_optical_frame',
        'min_depth_m': 0.15,
        'max_depth_m': 3.00,
    },
}


class MarkerDetector:
    """AprilTag/ArUco square detector with a metric centre estimate."""

    def __init__(self, dictionary_name, marker_id, size_m):
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('cv2.aruco is missing (install opencv-contrib)')
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name))
        self._parameters = cv2.aruco.DetectorParameters()
        self._parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._dictionary_name = dictionary_name
        self._marker_id = int(marker_id)
        half = 0.5 * float(size_m)
        # Marker frame: x right, y up, z out of the printed face. ArUco
        # returns corners in the matching clockwise order.
        self._object_points = np.array([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0],
        ], dtype=np.float64)

    def detect(self, gray, camera_matrix, distortion):
        """Return detection dict for the wanted marker, or None."""
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(
                self._dictionary, self._parameters)
            corners, ids, _ = detector.detectMarkers(gray)
        else:  # OpenCV < 4.7
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dictionary, parameters=self._parameters)
        if ids is None or corners is None:
            return None
        best = None
        for index, found_id in enumerate(np.asarray(ids).reshape(-1)):
            if self._marker_id >= 0 and int(found_id) != self._marker_id:
                continue
            points = np.asarray(
                corners[index], dtype=np.float64).reshape(4, 2)
            area = abs(cv2.contourArea(points.astype(np.float32)))
            if best is None or area > best[0]:
                best = (area, points, int(found_id))
        if best is None:
            return None
        _, points, found_id = best

        ok, rvec, tvec = cv2.solvePnP(
            self._object_points, points, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        try:
            rvec, tvec = cv2.solvePnPRefineLM(
                self._object_points, points, camera_matrix, distortion,
                rvec, tvec)
        except cv2.error:
            pass
        projected, _ = cv2.projectPoints(
            self._object_points, rvec, tvec, camera_matrix, distortion)
        reprojection = float(np.mean(np.linalg.norm(
            projected.reshape(4, 2) - points, axis=1)))
        return {
            'id': found_id,
            'corners': points,
            'centre': points.mean(axis=0),
            'tvec': tvec.reshape(3).astype(np.float64),
            'reprojection_px': reprojection,
        }


def _depth_metres(message, scale):
    """Return a float32 metre image from a depth message."""
    dtype = np.uint16 if message.encoding in ('16UC1', 'mono16') else np.float32
    raw = np.frombuffer(message.data, dtype=dtype)
    rows = message.step // np.dtype(dtype).itemsize
    image = raw[:rows * message.height].reshape(
        message.height, rows)[:, :message.width]
    if dtype is np.uint16:
        return image.astype(np.float32) * scale
    return image.astype(np.float32)


def _sample_depth(image, u, v, radius, minimum, min_depth_m, max_depth_m):
    """Median valid depth in a square patch; mirrors core.depth.sample_depth."""
    u0 = max(0, int(round(u)) - radius)
    u1 = min(image.shape[1], int(round(u)) + radius + 1)
    v0 = max(0, int(round(v)) - radius)
    v1 = min(image.shape[0], int(round(v)) + radius + 1)
    patch = image[v0:v1, u0:u1]
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid >= min_depth_m) & (valid <= max_depth_m)]
    if valid.size < max(1, int(minimum)):
        return None
    return float(np.median(valid)), int(valid.size)


def _transform_matrix(transform):
    """4x4 matrix from a geometry_msgs TransformStamped."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    matrix = np.eye(4)
    matrix[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class FixedPointCheck(Node):
    """Sample one camera + marker and write one CSV row per sample."""

    def __init__(self, args):
        super().__init__('fixed_point_check')
        self._args = args
        self._camera = CAMERAS[args.camera]
        self._detector = MarkerDetector(
            args.dictionary, args.marker_id, args.marker_size)
        self._rgb = None
        self._intrinsics = None
        self._distortion = None
        self._depth_ring = []
        self._buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self._listener = TransformListener(self._buffer, self)
        self.create_subscription(
            Image, self._camera['rgb_topic'], self._on_rgb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self._camera['depth_topic'], self._on_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self._camera['info_topic'], self._on_info,
            qos_profile_sensor_data)
        self._out = open(args.out, 'w', encoding='utf-8')
        self._out.write(
            't_ns,pose_seq,rgb_stamp_ns,depth_stamp_ns,tf_mode,depth_age_ms,'
            'u,v,depth_m,depth_valid,reprojection_px,'
            'cam_x,cam_y,cam_z,'
            'p_cam_x,p_cam_y,p_cam_z,p_base_x,p_base_y,p_base_z,'
            'pnp_cam_x,pnp_cam_y,pnp_cam_z,pnp_base_x,pnp_base_y,'
            'pnp_base_z,'
            'rot_r00,rot_r01,rot_r02,rot_r10,rot_r11,rot_r12,'
            'rot_r20,rot_r21,rot_r22\n')
        self._pose_seq = 0
        self._pose_origin = None
        self._samples = 0
        self._detections = 0
        self._tf_failures = 0
        self._last_depth_warning = 0.0

    # -- callbacks ---------------------------------------------------------
    def _on_rgb(self, message):
        self._rgb = message

    def _on_depth(self, message):
        self._depth_ring.append(message)
        if len(self._depth_ring) > 12:
            self._depth_ring.pop(0)

    def _on_info(self, message):
        self._intrinsics = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self._distortion = np.asarray(message.d, dtype=np.float64)

    # -- sampling ----------------------------------------------------------
    def sample(self):
        """Process the latest frame pair and append a CSV row."""
        if self._rgb is None or self._intrinsics is None:
            return None
        if not self._depth_ring:
            return None
        rgb_stamp = Time(
            nanoseconds=self._rgb.header.stamp.sec * 10 ** 9 +
            self._rgb.header.stamp.nanosec)
        tolerance = self._args.depth_tolerance_ms * 1e6
        depth_message = min(
            self._depth_ring,
            key=lambda message: abs(
                message.header.stamp.sec * 10 ** 9 +
                message.header.stamp.nanosec - rgb_stamp.nanoseconds))
        depth_stamp_ns = (depth_message.header.stamp.sec * 10 ** 9 +
                          depth_message.header.stamp.nanosec)
        age_ms = (rgb_stamp.nanoseconds - depth_stamp_ns) / 1e6
        if abs(age_ms) > self._args.depth_tolerance_ms:
            return None

        channels = 1 if self._rgb.encoding in ('mono8', '8UC1') else 3
        row_bytes = self._rgb.width * channels
        try:
            pixels = np.frombuffer(
                self._rgb.data, dtype=np.uint8).reshape(
                    self._rgb.height, self._rgb.step)[:, :row_bytes].reshape(
                        self._rgb.height, self._rgb.width, channels)
        except ValueError:
            return None
        if channels == 3 and self._rgb.encoding == 'rgb8':
            gray = cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY)
        elif channels == 3:
            gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY)
        else:
            gray = pixels[:, :, 0]
        detection = self._detector.detect(
            gray, self._intrinsics, self._distortion)
        if detection is None:
            return None
        self._detections += 1

        image = _depth_metres(depth_message, self._args.depth_scale)
        if image.shape[0] != self._rgb.height or \
                image.shape[1] != self._rgb.width:
            return None
        u, v = detection['centre']
        sampled = _sample_depth(
            image, u, v, self._args.depth_radius, 2,
            self._camera['min_depth_m'], self._camera['max_depth_m'])
        if sampled is None:
            return None
        depth_m, depth_valid = sampled

        fx = self._intrinsics[0, 0]
        fy = self._intrinsics[1, 1]
        cx = self._intrinsics[0, 2]
        cy = self._intrinsics[1, 2]
        p_cam = np.array([
            (u - cx) / fx * depth_m,
            (v - cy) / fy * depth_m,
            depth_m,
        ])
        pnp_cam = detection['tvec']

        transform, tf_mode = self._lookup_tf(depth_stamp_ns)
        if transform is None:
            return None
        origin = transform[:3, 3]
        if self._pose_origin is None or \
                np.linalg.norm(origin - self._pose_origin) > \
                self._args.pose_jump:
            self._pose_seq += 1
            self._pose_origin = origin
            print('POSE_CHANGE pose={} cam=({:+.3f}, {:+.3f}, {:+.3f})'.format(
                self._pose_seq, *origin), flush=True)

        def _to_base(point):
            return transform[:3, :3] @ point + transform[:3, 3]

        p_base = _to_base(p_cam)
        pnp_base = _to_base(pnp_cam)
        row = [
            self.get_clock().now().nanoseconds, self._pose_seq,
            rgb_stamp.nanoseconds, depth_stamp_ns, tf_mode,
            '{:.1f}'.format(age_ms),
            '{:.2f}'.format(u), '{:.2f}'.format(v),
            '{:.5f}'.format(depth_m), depth_valid,
            '{:.3f}'.format(detection['reprojection_px']),
            '{:.6f}'.format(origin[0]), '{:.6f}'.format(origin[1]),
            '{:.6f}'.format(origin[2]),
            '{:.6f}'.format(p_cam[0]), '{:.6f}'.format(p_cam[1]),
            '{:.6f}'.format(p_cam[2]),
            '{:.6f}'.format(p_base[0]), '{:.6f}'.format(p_base[1]),
            '{:.6f}'.format(p_base[2]),
            '{:.6f}'.format(pnp_cam[0]), '{:.6f}'.format(pnp_cam[1]),
            '{:.6f}'.format(pnp_cam[2]),
            '{:.6f}'.format(pnp_base[0]), '{:.6f}'.format(pnp_base[1]),
            '{:.6f}'.format(pnp_base[2]),
        ]
        # Full base<-camera rotation.  The hand-eye translation estimate needs
        # R_base_link4 per pose, which is R_base_cam @ R_link4_cam.T with the
        # nominally known (0.4 deg) hand-eye rotation.
        for row_index in range(3):
            for column_index in range(3):
                row.append('{:.8f}'.format(
                    transform[row_index, column_index]))
        self._out.write(','.join(str(value) for value in row) + '\n')
        self._out.flush()
        self._samples += 1
        print(
            'pose={} u={:.1f} v={:.1f} d={:.3f} '
            'depth=({:+.4f}, {:+.4f}, {:+.4f}) '
            'pnp=({:+.4f}, {:+.4f}, {:+.4f}) reproj={:.2f}px tf={}'.format(
                self._pose_seq, u, v, depth_m,
                p_base[0], p_base[1], p_base[2],
                pnp_base[0], pnp_base[1], pnp_base[2],
                detection['reprojection_px'], tf_mode), flush=True)
        return p_base

    def _lookup_tf(self, stamp_ns):
        """Look up base<-camera at the depth stamp, else latest."""
        try:
            transform = self._buffer.lookup_transform(
                self._args.base_frame, self._camera['camera_frame'],
                Time(nanoseconds=int(stamp_ns)),
                timeout=Duration(seconds=0.05))
            return _transform_matrix(transform), 'exact'
        except Exception:  # noqa: BLE001 - extrapolation is expected
            try:
                transform = self._buffer.lookup_transform(
                    self._args.base_frame, self._camera['camera_frame'],
                    Time(), timeout=Duration(seconds=0.1))
                return _transform_matrix(transform), 'latest'
            except Exception:  # noqa: BLE001 - fail closed
                self._tf_failures += 1
                return None, 'fail'

    def close(self):
        """Flush and close the CSV, print counters."""
        self._out.close()
        print('samples={} detections={} tf_failures={}'.format(
            self._samples, self._detections, self._tf_failures), flush=True)


def _self_test():
    """Synthetic marker at known poses must be recovered metrically."""
    detector = MarkerDetector('DICT_APRILTAG_36h11', 0, 0.06)
    intrinsics = np.array([
        [418.0542, 0.0, 308.7825],
        [0.0, 419.1000, 187.1342],
        [0.0, 0.0, 1.0],
    ])
    distortion = np.zeros(5)
    canvas_shape = (400, 640)
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, 0, 600, borderBits=1)
    # Trim the white border so the black square is exactly the marker size.
    dark = np.argwhere(marker < 128)
    v0, u0 = dark.min(axis=0)
    v1, u1 = dark.max(axis=0)
    square = marker[v0:v1 + 1, u0:u1 + 1]
    failures = 0
    for distance, angle_deg, centre in (
            (0.20, 0.0, (320, 200)),
            (0.35, 20.0, (200, 120)),
            (0.50, -35.0, (420, 260))):
        width = int(round(0.06 * intrinsics[0, 0] / distance))
        resized = cv2.resize(
            square, (width, width), interpolation=cv2.INTER_AREA)
        # Pad before rotating, otherwise the rotated corners are clipped.
        pad = int(math.ceil(width * math.sqrt(2.0))) + 4
        padded = np.full((pad, pad), 235, dtype=np.uint8)
        offset = (pad - width) // 2
        padded[offset:offset + width, offset:offset + width] = resized
        square_centre = offset + width / 2.0
        rotation = cv2.getRotationMatrix2D(
            (square_centre, square_centre), angle_deg, 1.0)
        rotated = cv2.warpAffine(
            padded, rotation, (pad, pad),
            flags=cv2.INTER_AREA, borderValue=235)
        canvas = np.full(canvas_shape, 235, dtype=np.uint8)
        x0 = int(round(centre[0] - square_centre))
        y0 = int(round(centre[1] - square_centre))
        canvas[y0:y0 + pad, x0:x0 + pad] = rotated
        u_true = x0 + square_centre
        v_true = y0 + square_centre
        found = detector.detect(canvas, intrinsics, distortion)
        if found is None:
            print('self-test FAIL: no detection at {:.2f} m'.format(distance))
            failures += 1
            continue
        # The rendered square is ``width`` pixels wide; that fixes the true
        # range to 0.06 * fx / width, independent of the rounded request.
        true_distance = 0.06 * intrinsics[0, 0] / width
        expected = np.array([
            (u_true - intrinsics[0, 2]) / intrinsics[0, 0] * true_distance,
            (v_true - intrinsics[1, 2]) / intrinsics[1, 1] * true_distance,
            true_distance,
        ])
        error = float(np.linalg.norm(found['tvec'] - expected))
        # Code-path check, not a metrology claim: a 71 px marker at 0.35 m
        # quantises to a few millimetres of range. Wrong corner order or a
        # wrong marker size shows up as tens of millimetres.
        status = 'ok' if error < 0.005 and \
            found['reprojection_px'] < 0.5 else 'FAIL'
        if status == 'FAIL':
            failures += 1
        print('self-test {} z={:.2f} angle={:+.0f}deg error={:.2f} mm '
              'reproj={:.2f} px'.format(
                  status, distance, angle_deg, error * 1000.0,
                  found['reprojection_px']))
    return 0 if failures == 0 else 1


def main(argv=None):
    """Run the self-test, or sample until duration/SIGINT."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--camera', choices=sorted(CAMERAS), default='wrist')
    parser.add_argument('--dictionary', default='DICT_APRILTAG_36h11')
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-size', type=float, default=0.06)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--depth-scale', type=float, default=0.001)
    parser.add_argument('--depth-radius', type=int, default=2)
    parser.add_argument('--depth-tolerance-ms', type=float, default=250.0)
    parser.add_argument('--pose-jump', type=float, default=0.01)
    parser.add_argument('--duration', type=float, default=180.0)
    parser.add_argument('--rate', type=float, default=3.0)
    parser.add_argument('--out', default='/tmp/fixed_point.csv')
    args = parser.parse_args(argv)
    if args.self_test:
        return _self_test()
    if ROS_IMPORT_ERROR is not None:
        print('ROS 2 imports failed: {}'.format(ROS_IMPORT_ERROR))
        return 2

    rclpy.init()
    node = FixedPointCheck(args)
    stop = {'flag': False}

    def _handle_sigint(_signum, _frame):
        stop['flag'] = True

    signal.signal(signal.SIGINT, _handle_sigint)
    deadline = time.time() + args.duration
    print('fixed-point check: {} camera, marker {} id {} size {:.3f} m, '
          '{:.1f} Hz, {:.0f} s -> {}'.format(
              args.camera, args.dictionary, args.marker_id,
              args.marker_size, args.rate, args.duration, args.out),
          flush=True)
    try:
        # Keep spinning between samples: a single spin_once per period leaves
        # the callback queue behind the streams, so the colour frame paired
        # with the depth ring ages past --depth-tolerance-ms and every sample
        # is dropped.
        next_sample = time.time()
        while not stop['flag'] and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now >= next_sample:
                node.sample()
                next_sample = max(next_sample + 1.0 / args.rate, now)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
