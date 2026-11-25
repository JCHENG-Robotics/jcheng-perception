#!/usr/bin/env python3
"""One combined status for the JCHENG perception stack.

The robot side is five systemd units plus ``jcheng-perception.target``; the
laptop side is the GPU detector and its reader-wedge watchdog.  This script
collects ``ActiveState`` / ``NRestarts`` / ``Result`` and the alert files from
both sides over SSH, writes ``status.json`` and ``status.txt`` under
``~/.local/state/jcheng-perception/``, and prints a short summary.

It is read-only: it never restarts anything.  The SSH call uses
``BatchMode=yes`` and a short timeout so a loaded robot does not block the
status timer.  A missing robot is reported as ``robot_unreachable`` rather than
treated as a laptop failure.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime

ROBOT_UNITS = (
    'jcheng-perception.target',
    'jcheng-perception-ingress@d435i_ext.service',
    'jcheng-perception-ingress@aurora_wrist.service',
    'jcheng-perception-fusion.service',
    'jcheng-perception-wrist-tf.service',
    'jcheng-perception-ext-tf.service',
)
LAPTOP_UNITS = (
    'jcheng-perception-gpu.service',
    'jcheng-perception-gpu-watchdog.service',
)
ROBOT_SSH = os.environ.get('JCHENG_ROBOT_SSH', 'pi@10.42.0.218')
ROBOT_KEY = os.path.expanduser(os.environ.get(
    'JCHENG_ROBOT_KEY', '~/.ssh/id_ed25519_armpi'))
ALERT_RECENT_S = float(
    os.environ.get('JCHENG_STATUS_ALERT_RECENT_S', '3600'))
# A unit that restarted this many times in the current boot is thrashing even
# if it is active right now (the external ingress does this during a DDS
# wedge).  Cumulative NRestarts, so this is a warning, not the start limit.
WARN_NRESTARTS = int(
    os.environ.get('JCHENG_STATUS_WARN_NRESTARTS', '5'))
STATE_DIR = os.path.join(
    os.environ.get('XDG_STATE_HOME', os.path.expanduser('~/.local/state')),
    'jcheng-perception')
STATUS_JSON = os.path.join(STATE_DIR, 'status.json')
STATUS_TXT = os.path.join(STATE_DIR, 'status.txt')

ROBOT_SCRIPT = r'''
set -u
for unit in {units}; do
    echo "@@UNIT $unit"
    systemctl show -p ActiveState -p NRestarts -p Result \
        -p ActiveEnterTimestamp "$unit" 2>/dev/null \
        || echo "ActiveState=unknown"
done
echo "@@LAST_ALERT"
sudo -n cat /var/lib/jcheng-perception/last-alert.txt 2>/dev/null || true
echo "@@ALERT_LOG"
sudo -n tail -n 5 /var/lib/jcheng-perception/alerts.log 2>/dev/null || true
echo "@@END"
'''.format(units=' '.join(ROBOT_UNITS))


def _run(command, *, input_text=None, timeout=20):
    try:
        result = subprocess.run(
            command, input=input_text, capture_output=True, text=True,
            timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        return 1, '', str(error)
    return result.returncode, result.stdout, result.stderr


def parse_show(text):
    """Parse ``systemctl show`` key=value lines into a dict."""
    values = {}
    for line in (text or '').splitlines():
        if '=' not in line:
            continue
        key, _, value = line.partition('=')
        values[key.strip()] = value.strip()
    return values


def unit_entry(show_text):
    """Return the fields reported for one unit."""
    values = parse_show(show_text)
    try:
        nrestarts = int(values.get('NRestarts', '0'))
    except ValueError:
        nrestarts = -1
    return {
        'active': values.get('ActiveState', 'unknown'),
        'nrestarts': nrestarts,
        'result': values.get('Result', 'unknown'),
        'started': values.get('ActiveEnterTimestamp', ''),
    }


def local_units():
    """Collect the laptop user units."""
    units = {}
    for unit in LAPTOP_UNITS:
        code, out, _err = _run([
            'systemctl', '--user', 'show',
            '-p', 'ActiveState', '-p', 'NRestarts', '-p', 'Result',
            '-p', 'ActiveEnterTimestamp', unit,
        ])
        units[unit] = unit_entry(out) if code == 0 else {
            'active': 'unavailable', 'nrestarts': -1,
            'result': 'unknown', 'started': '',
        }
    return units


def robot_units():
    """Collect the robot units and alert files over SSH."""
    command = [
        'ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
        '-o', 'IdentitiesOnly=yes',
    ]
    if os.path.isfile(ROBOT_KEY):
        command += ['-i', ROBOT_KEY]
    command += [ROBOT_SSH, 'bash -s']
    code, out, err = _run(command, input_text=ROBOT_SCRIPT, timeout=25)
    if code != 0:
        return False, {}, {'last': '', 'log_tail': [], 'error': err.strip()}
    units = {}
    last_alert = []
    alert_log = []
    section = None
    buffer = []
    current = None
    for line in out.splitlines():
        if line.startswith('@@UNIT '):
            if current is not None:
                units[current] = unit_entry('\n'.join(buffer))
            current = line.split(' ', 1)[1].strip()
            buffer = []
            section = 'unit'
        elif line == '@@LAST_ALERT':
            if current is not None:
                units[current] = unit_entry('\n'.join(buffer))
                current = None
                buffer = []
            section = 'last'
        elif line == '@@ALERT_LOG':
            section = 'log'
        elif line == '@@END':
            break
        elif section == 'unit':
            buffer.append(line)
        elif section == 'last':
            last_alert.append(line)
        elif section == 'log':
            alert_log.append(line)
    if current is not None:
        units[current] = unit_entry('\n'.join(buffer))
    return True, units, {
        'last': '\n'.join(last_alert).strip(),
        'log_tail': [line for line in alert_log if line.strip()],
    }


def read_file(path):
    try:
        with open(path, encoding='utf-8') as handle:
            return handle.read().strip()
    except OSError:
        return ''


def alert_age_s(text):
    """Return the age of an ``at=`` alert in seconds, or None."""
    for line in (text or '').splitlines():
        if not line.startswith('at='):
            continue
        raw = line.split('=', 1)[1].strip()
        try:
            then = datetime.fromisoformat(raw)
        except ValueError:
            return None
        now = datetime.now(then.tzinfo) if then.tzinfo else datetime.now()
        return max(0.0, (now - then).total_seconds())
    return None


def overall_status(robot_reachable, units, alerts):
    """Return ``ok``, ``degraded`` or ``robot_unreachable``."""
    if not robot_reachable:
        return 'robot_unreachable'
    if any(entry.get('active') != 'active' for entry in units.values()):
        return 'degraded'
    if any(
            int(entry.get('nrestarts', 0) or 0) >= WARN_NRESTARTS
            for entry in units.values()):
        return 'degraded'
    ages = [age for age in (
        alert_age_s(alerts.get('robot_last', '')),
        alert_age_s(alerts.get('laptop_last', '')),
        alert_age_s(alerts.get('laptop_watchdog', '')),
    ) if age is not None]
    if ages and min(ages) <= ALERT_RECENT_S:
        return 'degraded'
    return 'ok'


def warnings_for(robot_reachable, units, ages):
    """Return operator-facing warnings for restarts and recent alerts."""
    warnings = []
    if not robot_reachable:
        warnings.append('robot unreachable')
    for name, entry in sorted(units.items()):
        nrestarts = int(entry.get('nrestarts', 0) or 0)
        if nrestarts >= WARN_NRESTARTS:
            warnings.append(
                '{}: NRestarts={} (>= {})'.format(
                    name, nrestarts, WARN_NRESTARTS))
    for label in ('robot_age_s', 'laptop_age_s', 'laptop_watchdog_age_s'):
        age = ages.get(label)
        if age is not None and age <= ALERT_RECENT_S:
            warnings.append('{}: recent alert ({:.1f} h)'.format(
                label.replace('_age_s', ''), age / 3600.0))
    return warnings


def build_status():
    """Collect both sides and return the JSON-ready dict."""
    robot_reachable, robot, robot_alerts = robot_units()
    laptop = local_units()
    laptop_alerts = {
        'last': read_file(os.path.join(STATE_DIR, 'last-alert.txt')),
        'watchdog': read_file(
            os.path.join(STATE_DIR, 'last-watchdog-alert.txt')),
        'log_tail': read_file(
            os.path.join(STATE_DIR, 'alerts.log')).splitlines()[-5:],
    }
    all_units = {}
    all_units.update({'robot/' + name: entry for name, entry in robot.items()})
    all_units.update(
        {'laptop/' + name: entry for name, entry in laptop.items()})
    robot_age = alert_age_s(robot_alerts.get('last', ''))
    laptop_age = alert_age_s(laptop_alerts['last'])
    watchdog_age = alert_age_s(laptop_alerts['watchdog'])
    robot_alerts['age_s'] = robot_age
    laptop_alerts['age_s'] = laptop_age
    laptop_alerts['watchdog_age_s'] = watchdog_age
    alerts = {
        'robot_last': robot_alerts.get('last', ''),
        'robot_log_tail': robot_alerts.get('log_tail', []),
        'robot_age_s': robot_age,
        'laptop_last': laptop_alerts['last'],
        'laptop_watchdog': laptop_alerts['watchdog'],
        'laptop_log_tail': laptop_alerts['log_tail'],
        'laptop_age_s': laptop_age,
        'laptop_watchdog_age_s': watchdog_age,
    }
    warnings = warnings_for(robot_reachable, all_units, alerts)
    return {
        'checked_at': datetime.now().astimezone().isoformat(timespec='seconds'),
        'overall': overall_status(robot_reachable, all_units, alerts),
        'warnings': warnings,
        'robot': {
            'host': ROBOT_SSH,
            'reachable': robot_reachable,
            'units': robot,
            'alerts': robot_alerts,
        },
        'laptop': {
            'units': laptop,
            'alerts': laptop_alerts,
        },
    }


def summary(status):
    """Return a short human-readable summary."""
    lines = [
        'JCHENG perception status {}'.format(status['checked_at']),
        'robot {}: {}'.format(
            status['robot']['host'],
            'reachable' if status['robot']['reachable'] else 'UNREACHABLE'),
    ]
    for name, entry in status['robot']['units'].items():
        lines.append('  {:<55} {:<10} nrestarts={} result={}'.format(
            name, entry['active'], entry['nrestarts'], entry['result']))
    lines.append('laptop:')
    for name, entry in status['laptop']['units'].items():
        lines.append('  {:<55} {:<10} nrestarts={} result={}'.format(
            name, entry['active'], entry['nrestarts'], entry['result']))
    if status['robot']['alerts'].get('last'):
        age = status['robot']['alerts'].get('age_s')
        suffix = '' if age is None else ' (age {:.1f} h)'.format(age / 3600.0)
        lines.append('robot last alert:{}'.format(suffix))
        lines.extend('  ' + line
                     for line in status['robot']['alerts']['last'].splitlines())
    if status['laptop']['alerts'].get('last'):
        age = status['laptop']['alerts'].get('age_s')
        suffix = '' if age is None else ' (age {:.1f} h)'.format(age / 3600.0)
        lines.append('laptop last alert:{}'.format(suffix))
        lines.extend('  ' + line
                     for line in status['laptop']['alerts']['last'].splitlines())
    if status['laptop']['alerts'].get('watchdog'):
        age = status['laptop']['alerts'].get('watchdog_age_s')
        suffix = '' if age is None else ' (age {:.1f} h)'.format(age / 3600.0)
        lines.append('laptop watchdog alert:{}'.format(suffix))
        lines.extend('  ' + line for line in
                     status['laptop']['alerts']['watchdog'].splitlines())
    for warning in status.get('warnings', []):
        lines.append('WARN {}'.format(warning))
    lines.append('overall: {}'.format(status['overall']))
    return '\n'.join(lines)


def write_status(status):
    os.makedirs(STATE_DIR, exist_ok=True)
    with open(STATUS_JSON + '.tmp', 'w', encoding='utf-8') as handle:
        json.dump(status, handle, indent=1, sort_keys=True)
    os.replace(STATUS_JSON + '.tmp', STATUS_JSON)
    with open(STATUS_TXT + '.tmp', 'w', encoding='utf-8') as handle:
        handle.write(summary(status) + '\n')
    os.replace(STATUS_TXT + '.tmp', STATUS_TXT)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quiet', action='store_true',
                        help='write the files but do not print')
    parser.add_argument('--json', action='store_true',
                        help='print JSON instead of the short summary')
    args = parser.parse_args()
    status = build_status()
    write_status(status)
    if not args.quiet:
        if args.json:
            print(json.dumps(status, indent=1, sort_keys=True))
        else:
            print(summary(status))
    return 0


if __name__ == '__main__':
    sys.exit(main())
