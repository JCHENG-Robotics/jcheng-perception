"""Unit tests for the laptop GPU reader-wedge watchdog decision logic."""

import importlib.util
import sys
from pathlib import Path

WATCHDOG = (
    Path(__file__).resolve().parents[1]
    / 'deploy' / 'systemd' / 'laptop' / 'gpu-watchdog.py'
)


def _load():
    spec = importlib.util.spec_from_file_location('gpu_watchdog', WATCHDOG)
    module = importlib.util.module_from_spec(spec)
    # dataclasses looks the class module up in sys.modules while executing.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _detector(**overrides):
    defaults = {
        'start_time': 0.0,
        'warmup_s': 10.0,
        'stall_s': 60.0,
        'status_timeout_s': 30.0,
        'restart_limit': 5,
        'restart_window_s': 1800.0,
    }
    defaults.update(overrides)
    return _load().WedgeDetector(**defaults)


def test_warmup_then_healthy():
    detector = _detector()
    detector.on_status(5.0, {'ext': 5, 'wrist': 5})
    assert detector.decide(9.0, True) == (None, 'warm-up')
    assert detector.decide(20.0, True) == (None, 'ok')


def test_one_camera_frozen_restarts():
    detector = _detector()
    detector.on_status(10.0, {'ext': 10, 'wrist': 10})
    # Wrist keeps advancing; ext has not moved for 65 s.
    detector.on_status(75.0, {'ext': 10, 'wrist': 99})
    action, reason = detector.decide(80.0, True)
    assert action == 'restart'
    assert 'ext' in reason
    # The new process re-learns its counters before the next decision.
    assert detector.decide(85.0, True) == (None, 'warm-up after restart')


def test_both_frozen_is_not_restarted():
    detector = _detector()
    detector.on_status(10.0, {'ext': 10, 'wrist': 10})
    # Status is still fresh; only the counters are frozen (edge-side outage).
    detector.on_status(95.0, {'ext': 10, 'wrist': 10})
    action, reason = detector.decide(100.0, True)
    assert action is None
    assert 'all cameras frozen' in reason


def test_inactive_unit_is_left_alone():
    detector = _detector()
    detector.on_status(10.0, {'ext': 10, 'wrist': 10})
    assert detector.decide(100.0, False) == (None, 'unit inactive')


def test_stale_status_restarts():
    detector = _detector(warmup_s=5.0, status_timeout_s=30.0)
    action, reason = detector.decide(40.0, True)
    assert action == 'restart'
    assert 'stale' in reason


def test_restart_limit_alerts_and_stops():
    detector = _detector(
        warmup_s=0.0, stall_s=10.0, status_timeout_s=30.0,
        restart_limit=2, restart_window_s=600.0)
    detector.on_status(1.0, {'ext': 1, 'wrist': 1})
    detector.on_status(30.0, {'ext': 5, 'wrist': 30})
    detector.on_status(45.0, {'ext': 5, 'wrist': 60})
    assert detector.decide(50.0, True)[0] == 'restart'
    detector.on_status(60.0, {'ext': 5, 'wrist': 60})
    detector.on_status(75.0, {'ext': 5, 'wrist': 90})
    assert detector.decide(80.0, True)[0] == 'restart'
    detector.on_status(90.0, {'ext': 5, 'wrist': 90})
    detector.on_status(105.0, {'ext': 5, 'wrist': 120})
    action, reason = detector.decide(110.0, True)
    assert action == 'alert'
    assert 'restart limit' in reason
    # Within the alert cooldown it does not spam; keep the status fresh and
    # one camera advancing so the limit branch is the reason.
    detector.on_status(119.0, {'ext': 5, 'wrist': 140})
    assert detector.decide(120.0, True) == (None, 'restart limit reached')


def test_counter_reset_rebaselines():
    detector = _detector()
    detector.on_status(10.0, {'ext': 100, 'wrist': 10})
    # A GPU restart resets the counter; treat it as fresh data, not a wedge.
    detector.on_status(20.0, {'ext': 5, 'wrist': 20})
    detector.on_status(30.0, {'ext': 6, 'wrist': 30})
    assert detector.decide(40.0, True) == (None, 'ok')
