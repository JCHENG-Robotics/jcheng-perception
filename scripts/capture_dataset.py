#!/usr/bin/env python3
"""Record the perception uplink images into a dataset session.

Subscribes BEST_EFFORT to ``/perception/uplink/<camera>/image_compressed`` and
writes the JPEG bytes unchanged (no decode or re-encode). Frames are named by
their capture stamp; ``manifest.csv`` and ``session.yaml`` record provenance
for the labeler and the training split.

Run in a Humble environment with the wired CycloneDDS config, for example
inside the GPU container (the dataset directory must be mounted):

    python3 scripts/capture_dataset.py \
      --out /data/raw --session 20261125_cube_01 --duration 20 \
      --notes 'cube on the table, view pose'

The host directory layout is ``<out>/<session>/<camera>/<stamp_ns>.jpg``. No
dataset is ever written inside this repository.
"""

import argparse
import csv
import datetime
import pathlib
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def _qos():
    qos = QoSProfile(depth=5)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    return qos


class DatasetCapture(Node):
    """Write compressed frames from the perception uplink to disk."""

    def __init__(self, out_dir, cameras, max_rate, notes):
        super().__init__('dataset_capture')
        self.out_dir = pathlib.Path(out_dir)
        self.cameras = cameras
        self.max_rate = max_rate
        self.notes = notes
        self.dirs = {}
        self.last_ns = {camera: 0 for camera in cameras}
        self.counts = {camera: 0 for camera in cameras}
        self.rows = []
        self.started = time.time()
        self.start_iso = datetime.datetime.now().astimezone().isoformat()
        for camera in cameras:
            directory = self.out_dir / camera
            directory.mkdir(parents=True, exist_ok=True)
            self.dirs[camera] = directory
            topic = '/perception/uplink/{}/image_compressed'.format(camera)
            self.create_subscription(
                CompressedImage, topic,
                lambda message, camera=camera: self.on_image(camera, message),
                _qos())
            self.get_logger().info('recording {} from {}'.format(camera, topic))
        self.create_timer(2.0, self.report)

    def on_image(self, camera, message):
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + \
            int(message.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = time.time_ns()
        if self.max_rate > 0 and \
                stamp_ns - self.last_ns[camera] < 1e9 / self.max_rate:
            return
        path = self.dirs[camera] / '{}.jpg'.format(stamp_ns)
        if path.exists():
            path = self.dirs[camera] / '{}_{}.jpg'.format(
                stamp_ns, self.counts[camera])
        data = bytes(message.data)
        path.write_bytes(data)
        self.last_ns[camera] = stamp_ns
        self.counts[camera] += 1
        self.rows.append({
            'camera': camera,
            'stamp_ns': stamp_ns,
            'file': str(path.relative_to(self.out_dir)),
            'format': message.format,
            'bytes': len(data),
            'frame_id': message.header.frame_id,
        })

    def report(self):
        summary = ', '.join(
            '{}={}'.format(camera, self.counts[camera])
            for camera in self.cameras)
        self.get_logger().info('recorded: {}'.format(summary))

    def finish(self):
        self.out_dir.mkdir(parents=True, exist_ok=True)
        manifest = self.out_dir / 'manifest.csv'
        with manifest.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream, fieldnames=[
                    'camera', 'stamp_ns', 'file', 'format', 'bytes',
                    'frame_id'])
            writer.writeheader()
            writer.writerows(self.rows)
        session = {
            'session': self.out_dir.name,
            'notes': self.notes,
            'started': self.start_iso,
            'finished': datetime.datetime.now().astimezone().isoformat(),
            'duration_s': round(time.time() - self.started, 3),
            'cameras': list(self.cameras),
            'counts': dict(self.counts),
            'topics': {
                camera: '/perception/uplink/{}/image_compressed'.format(camera)
                for camera in self.cameras
            },
            'layout': '<session>/<camera>/<stamp_ns>.jpg',
        }
        (self.out_dir / 'session.yaml').write_text(
            yaml.safe_dump(session, sort_keys=False), encoding='utf-8')
        self.get_logger().info(
            'session {} finished: {}'.format(
                self.out_dir.name, self.counts))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--out', default=str(pathlib.Path.home() /
                             'jcheng/perception_datasets/raw'),
        help='dataset root; a session directory is created below it')
    parser.add_argument(
        '--session', default=None,
        help='session name (default: YYYYmmdd_HHMMSS)')
    parser.add_argument(
        '--cameras', default='d435i_ext,aurora_wrist',
        help='comma-separated camera ids')
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument(
        '--max-rate', type=float, default=0.0,
        help='per-camera save rate limit in Hz (0 = every frame)')
    parser.add_argument('--notes', default='')
    args = parser.parse_args()

    session = args.session or datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    cameras = [item.strip() for item in args.cameras.split(',') if item.strip()]
    if not cameras:
        parser.error('--cameras is empty')
    out_dir = pathlib.Path(args.out).expanduser() / session

    rclpy.init()
    node = DatasetCapture(out_dir, cameras, args.max_rate, args.notes)
    deadline = time.monotonic() + max(0.0, args.duration)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()
    if not any(node.counts.values()):
        print('FAIL: no frames received; check the camera and DDS link')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
