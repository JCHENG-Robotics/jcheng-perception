"""
Timestamp-indexed ring buffers.

The distributed pipeline matches detections back to the depth image and the TF
sample *taken at the same acquisition time*. ROS messages only carry the
message, not the ring index, so every match is a time query. These buffers are
small, ordered and bounded; out-of-order delivery is handled by sorted insert.
"""

import bisect
from collections import deque
from typing import Generic, List, Optional, Tuple, TypeVar

import numpy as np

from .geometry import quaternion_slerp

T = TypeVar('T')


class TimestampRingBuffer(Generic[T]):
    """A bounded, time-ordered container with nearest-stamp lookup."""

    def __init__(self, max_items=512, max_age_ns=5_000_000_000):
        """Create a buffer bounded by item count and age."""
        if max_items <= 0:
            raise ValueError('max_items must be positive')
        self.max_items = int(max_items)
        self.max_age_ns = int(max_age_ns)
        self._stamps: List[int] = []
        self._items: List[T] = []

    def __len__(self):
        """Return the number of retained items."""
        return len(self._stamps)

    def clear(self):
        """Drop all retained items."""
        self._stamps.clear()
        self._items.clear()

    @property
    def stamps(self):
        """Return a copy of the retained timestamps."""
        return list(self._stamps)

    def add(self, stamp_ns, item):
        """Insert an item, replacing an existing sample with the same stamp."""
        stamp_ns = int(stamp_ns)
        index = bisect.bisect_left(self._stamps, stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == stamp_ns:
            self._items[index] = item
        else:
            self._stamps.insert(index, stamp_ns)
            self._items.insert(index, item)
        while len(self._stamps) > self.max_items:
            self._stamps.pop(0)
            self._items.pop(0)

    def prune(self, now_ns):
        """Drop samples older than ``max_age_ns`` relative to ``now_ns``."""
        cutoff = int(now_ns) - self.max_age_ns
        count = 0
        while count < len(self._stamps) and self._stamps[count] < cutoff:
            count += 1
        if count:
            del self._stamps[:count]
            del self._items[:count]

    def nearest(self, stamp_ns, max_delta_ns=None):
        """Return ``(stamp, item)`` nearest to ``stamp_ns`` within tolerance."""
        if not self._stamps:
            return None
        stamp_ns = int(stamp_ns)
        index = bisect.bisect_left(self._stamps, stamp_ns)
        candidates = []
        if index < len(self._stamps):
            candidates.append(index)
        if index > 0:
            candidates.append(index - 1)
        best_index = min(
            candidates,
            key=lambda item: abs(self._stamps[item] - stamp_ns),
        )
        delta = abs(self._stamps[best_index] - stamp_ns)
        if max_delta_ns is not None and delta > int(max_delta_ns):
            return None
        return self._stamps[best_index], self._items[best_index]

    def latest(self):
        """Return the newest ``(stamp, item)`` pair, or None."""
        if not self._stamps:
            return None
        return self._stamps[-1], self._items[-1]

    def between(self, start_ns, end_ns):
        """Return all ``(stamp, item)`` pairs in the closed time interval."""
        start = bisect.bisect_left(self._stamps, int(start_ns))
        end = bisect.bisect_right(self._stamps, int(end_ns))
        return list(zip(self._stamps[start:end], self._items[start:end]))


class PoseRingBuffer(TimestampRingBuffer[np.ndarray]):
    """Ring of 4x4 base<-camera transforms with interpolation."""

    def pose_at(self, stamp_ns, max_gap_ns=80_000_000,
                max_delta_ns=None):
        """
        Return the interpolated pose at ``stamp_ns``.

        Exact samples win. Between two samples, translation is linear and
        rotation is slerped; a gap larger than ``max_gap_ns`` is rejected so a
        stale pose is never silently paired with a fresh image.
        """
        if not self._stamps:
            return None
        stamp_ns = int(stamp_ns)
        index = bisect.bisect_left(self._stamps, stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == stamp_ns:
            return self._items[index].copy()
        if index == 0:
            delta = self._stamps[0] - stamp_ns
            if max_delta_ns is not None and delta <= int(max_delta_ns):
                return self._items[0].copy()
            return None
        if index == len(self._stamps):
            delta = stamp_ns - self._stamps[-1]
            if max_delta_ns is not None and delta <= int(max_delta_ns):
                return self._items[-1].copy()
            return None

        stamp_before = self._stamps[index - 1]
        stamp_after = self._stamps[index]
        gap = stamp_after - stamp_before
        if gap > int(max_gap_ns) or gap <= 0:
            return None
        fraction = float(stamp_ns - stamp_before) / float(gap)
        return _interpolate_transform(
            self._items[index - 1],
            self._items[index],
            fraction,
        )


def _interpolate_transform(before, after, fraction):
    """Blend two 4x4 transforms."""
    before = np.asarray(before, dtype=np.float64)
    after = np.asarray(after, dtype=np.float64)
    blended = np.eye(4, dtype=np.float64)
    blended[:3, 3] = (
        (1.0 - fraction) * before[:3, 3] + fraction * after[:3, 3]
    )
    from .geometry import quaternion_from_matrix

    start = quaternion_from_matrix(before[:3, :3])
    end = quaternion_from_matrix(after[:3, :3])
    quaternion = quaternion_slerp(start, end, fraction)
    from .geometry import matrix_from_quaternion

    blended[:3, :3] = matrix_from_quaternion(quaternion)
    return blended


class FrameRing:
    """Convenience wrapper combining frame metadata and depth buffers."""

    def __init__(self, max_items=512, max_age_ns=5_000_000_000):
        """Create metadata and depth rings with the same bounds."""
        self.meta = TimestampRingBuffer(
            max_items=max_items, max_age_ns=max_age_ns)
        self.depth = TimestampRingBuffer(
            max_items=max_items, max_age_ns=max_age_ns)

    def add_meta(self, stamp_ns, meta):
        """Store one frame metadata record."""
        self.meta.add(stamp_ns, meta)

    def add_depth(self, stamp_ns, depth_frame):
        """Store one depth frame."""
        self.depth.add(stamp_ns, depth_frame)

    def prune(self, now_ns):
        """Prune both rings."""
        self.meta.prune(now_ns)
        self.depth.prune(now_ns)

    def __len__(self):
        """Return the number of metadata records."""
        return len(self.meta)


class LatencyWindow:
    """Small rolling window for latency and rate statistics."""

    def __init__(self, size=120):
        """Create a window of ``size`` samples."""
        self._samples = deque(maxlen=max(1, int(size)))
        self._stamps = deque(maxlen=max(1, int(size)))

    def add(self, value, stamp_ns=None):
        """Append one latency sample and an optional event timestamp."""
        self._samples.append(float(value))
        if stamp_ns is not None:
            self._stamps.append(int(stamp_ns))

    def mean(self):
        """Return the mean latency, or 0.0 when empty."""
        if not self._samples:
            return 0.0
        return float(sum(self._samples) / len(self._samples))

    def rate_hz(self):
        """Estimate the event rate from the retained timestamps."""
        if len(self._stamps) < 2:
            return 0.0
        span_ns = self._stamps[-1] - self._stamps[0]
        if span_ns <= 0:
            return 0.0
        return (len(self._stamps) - 1) * 1e9 / float(span_ns)

    def __len__(self):
        """Return the number of samples."""
        return len(self._samples)


# A gap longer than this is treated as a clock jump or a long camera pause, not
# as a normal inter-frame interval.
UPLINK_CLOCK_JUMP_NS = 10_000_000_000


def uplink_allowed(now_ns, last_ns, min_interval_ns):
    """Return ``(allowed, new_last_ns)`` for the per-camera uplink limiter.

    The Pi clock can jump backwards after an NTP correction. A plain
    ``now - last < interval`` check then leaves ``last`` in the future and
    silently skips that camera's uplinks until the clock catches up. A
    backwards jump (or a gap longer than ``UPLINK_CLOCK_JUMP_NS``) resets the
    baseline and allows the frame.
    """
    now_ns = int(now_ns)
    last_ns = int(last_ns)
    interval_ns = max(0, int(min_interval_ns))
    elapsed_ns = now_ns - last_ns
    if elapsed_ns < 0 or elapsed_ns > UPLINK_CLOCK_JUMP_NS:
        return True, now_ns
    if elapsed_ns < interval_ns:
        return False, last_ns
    return True, now_ns


def link_stalled(now_ns, last_change_ns, had_data, timeout_ns,
                 cooldown_until_ns=0):
    """Return True when a sensor link should be considered stalled."""
    if not had_data:
        return False
    now_ns = int(now_ns)
    last_change_ns = int(last_change_ns)
    timeout_ns = max(1, int(timeout_ns))
    if now_ns < int(cooldown_until_ns):
        return False
    return now_ns - last_change_ns >= timeout_ns
