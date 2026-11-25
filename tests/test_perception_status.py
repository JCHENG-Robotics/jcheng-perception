"""Unit tests for the combined perception status aggregator."""

import importlib.util
import sys
from datetime import datetime, timedelta
from pathlib import Path

STATUS = (
    Path(__file__).resolve().parents[1]
    / 'deploy' / 'systemd' / 'laptop' / 'jcheng-perception-status.py'
)


def _load():
    spec = importlib.util.spec_from_file_location('perception_status', STATUS)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_show_and_unit_entry():
    module = _load()
    entry = module.unit_entry(
        'ActiveState=active\n'
        'NRestarts=3\n'
        'Result=success\n'
        'ActiveEnterTimestamp=Wed 2026-11-25 15:00:00 HKT\n')
    assert entry['active'] == 'active'
    assert entry['nrestarts'] == 3
    assert entry['result'] == 'success'
    assert '2026-11-25' in entry['started']
    assert module.parse_show('A=1\nB=2') == {'A': '1', 'B': '2'}


def test_alert_age_and_missing():
    module = _load()
    now = datetime.now().astimezone()
    fresh = 'at={}'.format(now.isoformat(timespec='seconds'))
    age = module.alert_age_s(fresh)
    assert age is not None and age < 10
    assert module.alert_age_s('at=2000-01-01T00:00:00+00:00') > 3600
    assert module.alert_age_s('') is None
    assert module.alert_age_s('not an alert') is None


def _units(active='active', nrestarts=0):
    return {
        'robot/jcheng-perception.target': {
            'active': active, 'nrestarts': nrestarts},
        'robot/jcheng-perception-fusion.service': {
            'active': active, 'nrestarts': nrestarts},
        'laptop/jcheng-perception-gpu.service': {
            'active': active, 'nrestarts': nrestarts},
    }


def test_overall_status():
    module = _load()
    assert module.overall_status(False, {}, {}) == 'robot_unreachable'
    assert module.overall_status(True, _units(), {}) == 'ok'
    assert module.overall_status(
        True, _units(active='failed'), {}) == 'degraded'
    assert module.overall_status(
        True, _units(nrestarts=9), {}) == 'degraded'
    recent = 'at={}'.format(
        datetime.now().astimezone().isoformat(timespec='seconds'))
    assert module.overall_status(
        True, _units(), {'robot_last': recent}) == 'degraded'
    old = 'at=2000-01-01T00:00:00+00:00'
    assert module.overall_status(
        True, _units(), {'robot_last': old}) == 'ok'


def test_warnings_for_restarts_and_alerts():
    module = _load()
    warnings = module.warnings_for(
        True, _units(nrestarts=9),
        {'robot_age_s': 120.0, 'laptop_age_s': None})
    text = '\n'.join(warnings)
    assert 'NRestarts=9' in text
    assert 'robot: recent alert' in text
    assert 'laptop: recent alert' not in text
    assert 'robot unreachable' in module.warnings_for(False, {}, {})[0]


def test_summary_contains_both_sides():
    module = _load()
    status = {
        'checked_at': '2026-11-25T15:00:00+08:00',
        'overall': 'ok',
        'warnings': [
            'robot/jcheng-perception-fusion.service: NRestarts=9 (>= 5)'],
        'robot': {
            'host': 'pi@10.42.0.218',
            'reachable': True,
            'units': {'jcheng-perception-fusion.service': {
                'active': 'active', 'nrestarts': 0,
                'result': 'success', 'started': ''}},
            'alerts': {'last': '', 'log_tail': []},
        },
        'laptop': {
            'units': {'jcheng-perception-gpu.service': {
                'active': 'active', 'nrestarts': 0,
                'result': 'success', 'started': ''}},
            'alerts': {'last': '', 'watchdog': '', 'log_tail': []},
        },
    }
    text = module.summary(status)
    assert 'robot pi@10.42.0.218: reachable' in text
    assert 'jcheng-perception-fusion.service' in text
    assert 'jcheng-perception-gpu.service' in text
    assert 'WARN robot/jcheng-perception-fusion.service: NRestarts=9' in text
    assert text.rstrip().endswith('overall: ok')
