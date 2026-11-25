#!/usr/bin/env python3
"""Restart the laptop GPU detector when one of its DDS readers wedges.

The known failure after an edge ingress restart: the GPU process stays alive
and keeps publishing ``GpuStatus``, but ``frames_received`` for one camera
stops advancing.  systemd cannot see that (the unit is still ``active``), so
this watchdog subscribes to ``/perception/gpu_status`` and restarts the user
unit when:

* one camera's counter is frozen for ``JCHENG_GPU_STALL_S`` (default 60 s)
  while another camera is still advancing; or
* ``GpuStatus`` itself has been absent for ``JCHENG_GPU_STATUS_TIMEOUT_S``
  (default 30 s) while the unit is active.

Both cameras frozen is treated as an edge-side outage (uplinks down) and does
**not** restart the GPU; a restart cannot fix that.  Restarts are capped at
``JCHENG_GPU_WATCHDOG_RESTARTS`` (default 5) per
``JCHENG_GPU_WATCHDOG_WINDOW_S`` (default 1800 s), after which an alert is
written and the watchdog stops acting until the window expires.

The decision logic is a pure class (``WedgeDetector``) so it can be tested
without ROS.  The ROS imports live inside ``main``.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from dataclasses import dataclass

UNIT = 'jcheng-perception-gpu.service'
STATE_DIR = os.path.join(
    os.environ.get('XDG_STATE_HOME', os.path.expanduser('~/.local/state')),
    'jcheng-perception')
STALL_S = float(os.environ.get('JCHENG_GPU_STALL_S', '60'))
STATUS_TIMEOUT_S = float(
    os.environ.get('JCHENG_GPU_STATUS_TIMEOUT_S', '30'))
WARMUP_S = float(os.environ.get('JCHENG_GPU_WATCHDOG_WARMUP_S', '90'))
RESTART_LIMIT = int(os.environ.get('JCHENG_GPU_WATCHDOG_RESTARTS', '5'))
RESTART_WINDOW_S = float(
    os.environ.get('JCHENG_GPU_WATCHDOG_WINDOW_S', '1800'))
TICK_S = float(os.environ.get('JCHENG_GPU_WATCHDOG_TICK_S', '10'))


@dataclass
class CameraState:
    """Last counter value and the time it last advanced."""

    last_count: int | None = None
    last_change: float = 0.0


class WedgeDetector:
    """Pure state machine for one-camera reader wedges."""

    def __init__(
            self, *, stall_s=STALL_S, status_timeout_s=STATUS_TIMEOUT_S,
            warmup_s=WARMUP_S, restart_limit=RESTART_LIMIT,
            restart_window_s=RESTART_WINDOW_S, start_time=None):
        self.stall_s = float(stall_s)
        self.status_timeout_s = float(status_timeout_s)
        self.warmup_s = float(warmup_s)
        self.restart_limit = int(restart_limit)
        self.restart_window_s = float(restart_window_s)
        self.started = (
            time.monotonic() if start_time is None else float(start_time))
        self.last_status = 0.0
        self.last_restart = -1e9
        self.last_alert = -1e9
        self.cameras: dict[str, CameraState] = {}
        self.restarts: list[float] = []
        self.last_reason = ''

    def on_status(self, now, counts):
        """Feed one ``GpuStatus`` sample (camera id -> frames_received)."""
        now = float(now)
        counts = {
            str(camera): int(count)
            for camera, count in (counts or {}).items()
            if camera
        }
        if not counts:
            return
        self.last_status = now
        for camera, count in counts.items():
            state = self.cameras.setdefault(camera, CameraState())
            if state.last_count is None:
                state.last_count = count
                state.last_change = now
            elif count != state.last_count:
                # Advance or a GPU restart resetting the counter; both are
                # evidence that this reader is still moving data.
                state.last_count = count
                state.last_change = now

    def decide(self, now, unit_active):
        """Return ``(action, reason)``; action is None, 'restart' or 'alert'."""
        now = float(now)
        if not unit_active:
            # Do not keep the status/counter state from before the unit went
            # down: on a manual start the old last_status would already be
            # stale and the watchdog would restart the fresh process at once.
            # Anchor a full status window to the moment the unit is seen down.
            self.last_status = now
            self.cameras.clear()
            return None, 'unit inactive'
        if now - self.started < self.warmup_s:
            return None, 'warm-up'
        if now - self.last_restart < self.warmup_s:
            return None, 'warm-up after restart'
        if (self.last_status == 0.0
                or now - self.last_status > self.status_timeout_s):
            return self._action(now, 'gpu_status stale')
        if not self.cameras:
            return None, 'no camera counters yet'
        frozen = sorted(
            camera for camera, state in self.cameras.items()
            if now - state.last_change >= self.stall_s)
        if not frozen:
            return None, 'ok'
        if len(frozen) == len(self.cameras):
            return None, 'all cameras frozen (edge-side outage?)'
        return self._action(now, 'frozen: ' + ','.join(frozen))

    def _action(self, now, reason):
        self.restarts = [
            stamp for stamp in self.restarts
            if now - stamp < self.restart_window_s
        ]
        if len(self.restarts) >= self.restart_limit:
            if now - self.last_alert >= 300.0:
                self.last_alert = now
                return 'alert', (
                    '{}; restart limit {}/{}s reached'.format(
                        reason, self.restart_limit,
                        int(self.restart_window_s)))
            return None, 'restart limit reached'
        self.restarts.append(now)
        self.last_restart = now
        # The new process starts its counters from zero; re-learn them before
        # the next decision.
        self.last_status = 0.0
        self.cameras.clear()
        return 'restart', reason


def _unit_active():
    """Return True only while the GPU unit is in the ``active`` state."""
    try:
        result = subprocess.run(
            ['systemctl', '--user', 'is-active', UNIT],
            capture_output=True, text=True, timeout=10, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.stdout.strip() == 'active'


def _restart_unit():
    try:
        result = subprocess.run(
            ['systemctl', '--user', 'restart', UNIT],
            capture_output=True, text=True, timeout=45, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        return False, str(error)
    return result.returncode == 0, (result.stdout + result.stderr).strip()


def _write_alert(reason, detector):
    os.makedirs(STATE_DIR, exist_ok=True)
    stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    frozen = ', '.join(sorted(detector.cameras)) or 'none'
    action = ('systemctl --user reset-failed {} && '
              'systemctl --user start {}'.format(UNIT, UNIT))
    message = (
        'ALERT GPU watchdog: {}; camera_state={}; restart budget '
        '{}/{}s exhausted; after fixing the GPU/edge run: {}'.format(
            reason, frozen, detector.restart_limit,
            int(detector.restart_window_s), action))
    path = os.path.join(STATE_DIR, 'last-watchdog-alert.txt')
    with open(path + '.tmp', 'w', encoding='utf-8') as handle:
        handle.write('at={}\nreason={}\naction={}\n'.format(
            stamp, reason, action))
    os.replace(path + '.tmp', path)
    with open(os.path.join(STATE_DIR, 'watchdog-alerts.log'),
              'a', encoding='utf-8') as handle:
        handle.write('{} {}\n'.format(stamp, message))
    try:
        subprocess.run(
            ['logger', '-t', 'jcheng-perception-gpu-watchdog',
             '-p', 'user.err', '--', message],
            timeout=5, check=False)
    except (OSError, subprocess.TimeoutExpired):
        pass
    return message


def main():
    """Run the watchdog as a ROS node (called by the user service)."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from jcheng_perception_msgs.msg import GpuStatus

    rclpy.init()
    node = Node('jcheng_gpu_watchdog')
    detector = WedgeDetector()
    armed = False

    qos = QoSProfile(
        depth=10,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    def on_status(message):
        counts = {
            camera: count
            for camera, count in zip(
                message.camera_ids, message.frames_received)
        }
        detector.on_status(time.monotonic(), counts)

    node.create_subscription(
        GpuStatus, '/perception/gpu_status', on_status, qos)

    def tick():
        nonlocal armed
        now = time.monotonic()
        if not armed and now - detector.started >= detector.warmup_s:
            armed = True
            node.get_logger().info(
                'watchdog armed: stall={:.0f}s status_timeout={:.0f}s '
                'restart_budget={}/{}s'.format(
                    detector.stall_s, detector.status_timeout_s,
                    detector.restart_limit,
                    int(detector.restart_window_s)))
        action, reason = detector.decide(now, _unit_active())
        if reason != detector.last_reason:
            detector.last_reason = reason
            node.get_logger().info('state: {}'.format(reason))
        if action == 'restart':
            node.get_logger().warn(
                'restarting {} ({})'.format(UNIT, reason))
            ok, output = _restart_unit()
            if ok:
                node.get_logger().warn('restart requested')
            else:
                node.get_logger().error(
                    'restart failed: {}'.format(output or 'unknown'))
        elif action == 'alert':
            node.get_logger().error(_write_alert(reason, detector))

    node.create_timer(TICK_S, tick)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
