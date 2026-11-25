#!/usr/bin/env python3
"""Measure the camera streams the perception ingress consumes.

Run this before a perception session (inside the Humble container or with a
Humble environment). It subscribes BEST_EFFORT to the Aurora RGB/depth topics
and checks that they are near the driver's nominal ~12 Hz / ~8-11 Hz.

The Aurora driver degrades after a long uptime: on 2026-11-25 the wrist ingress
saw 7.95 Hz RGB but only 2.64 Hz depth after 12 h, and a driver restart
restored 10.92 / 10.83 Hz. If the depth rate is below the threshold, restart
the driver:

    sudo systemctl restart armpi-vision     # ~2.5 min readiness poll
    docker exec ArmPiUltra pkill -f vision.launch.py   # only if needed

The ingress watchdog respawns its process after the driver returns.

Exit codes: 0 both streams healthy, 1 any stream missing/low, 2 rclpy missing.
"""

import argparse
import sys
import time

DEFAULT_TOPICS = ('/depth_cam/rgb/image_raw', '/depth_cam/depth/image_raw')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--min-rgb', type=float, default=6.0)
    parser.add_argument('--min-depth', type=float, default=5.0)
    parser.add_argument(
        '--topic', action='append', dest='topics',
        help='override the measured topics (repeatable)')
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
    except ImportError as error:
        print('FAIL: rclpy/sensor_msgs not available: {}'.format(error),
              file=sys.stderr)
        return 2

    topics = tuple(args.topics or DEFAULT_TOPICS)
    rclpy.init()
    node = Node('camera_rate_probe')
    counts = {topic: 0 for topic in topics}
    qos = QoSProfile(depth=5)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    for topic in topics:
        node.create_subscription(
            Image, topic,
            lambda message, topic=topic: counts.__setitem__(
                topic, counts[topic] + 1),
            qos)

    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    rates = {topic: counts[topic] / args.duration for topic in topics}
    for topic in topics:
        print('{}: {:.2f} Hz ({} frames)'.format(
            topic, rates[topic], counts[topic]))

    failures = []
    if topics and rates[topics[0]] < args.min_rgb:
        failures.append('rgb {:.2f} < {:.1f} Hz'.format(
            rates[topics[0]], args.min_rgb))
    if len(topics) > 1 and rates[topics[1]] < args.min_depth:
        failures.append('depth {:.2f} < {:.1f} Hz'.format(
            rates[topics[1]], args.min_depth))
    if failures:
        print('FAIL: {}'.format(', '.join(failures)))
        print('Restart the Aurora driver: sudo systemctl restart armpi-vision')
        return 1
    print('OK: streams healthy')
    return 0


if __name__ == '__main__':
    sys.exit(main())
