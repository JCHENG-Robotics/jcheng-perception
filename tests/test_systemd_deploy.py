"""Static checks for the systemd deployment of the split stack.

The units run on the robot, so these tests cannot start them; they assert the
contract that matters: one process per unit, the watchdog's clean exit is
covered by Restart=always, the ingress restart budget is finite, and a failed
unit triggers the alert template.  Syntax is checked with bash -n and, when
available, systemd-analyze verify.
"""

import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

DEPLOY = Path(__file__).resolve().parents[1] / 'deploy' / 'systemd'
INGRESS_UNIT = DEPLOY / 'jcheng-perception-ingress@.service'
FUSION_UNIT = DEPLOY / 'jcheng-perception-fusion.service'
ALERT_UNIT = DEPLOY / 'jcheng-perception-alert@.service'
TARGET_UNIT = DEPLOY / 'jcheng-perception.target'
WRIST_TF_UNIT = DEPLOY / 'jcheng-perception-wrist-tf.service'
EXT_TF_UNIT = DEPLOY / 'jcheng-perception-ext-tf.service'
SCRIPTS = (
    DEPLOY / 'ingress-run.sh',
    DEPLOY / 'fusion-run.sh',
    DEPLOY / 'wrist-tf-run.sh',
    DEPLOY / 'ext-tf-run.sh',
    DEPLOY / 'alert.sh',
    DEPLOY / 'install-split-systemd.sh',
)


def unit_text(path):
    """Return a unit file as text."""
    return path.read_text(encoding='utf-8')


def test_expected_units_and_scripts_exist():
    for path in (INGRESS_UNIT, FUSION_UNIT, ALERT_UNIT, TARGET_UNIT,
                 WRIST_TF_UNIT, EXT_TF_UNIT,
                 DEPLOY / 'perception.env', *SCRIPTS):
        assert path.is_file(), path
    for script in SCRIPTS:
        assert script.stat().st_mode & 0o111, script


def test_target_starts_the_three_processes_and_required_tf():
    text = unit_text(TARGET_UNIT)
    assert 'Wants=jcheng-perception-ingress@d435i_ext.service' in text
    assert 'Wants=jcheng-perception-ingress@aurora_wrist.service' in text
    assert 'Wants=jcheng-perception-fusion.service' in text
    # Without these the wrist view is dropped and the external chain has no
    # base_link; they used to be hand-started with the launch.
    assert 'Wants=jcheng-perception-wrist-tf.service' in text
    assert 'Wants=jcheng-perception-ext-tf.service' in text
    assert 'WantedBy=multi-user.target' in text


def test_ingress_restart_policy_and_budget():
    text = unit_text(INGRESS_UNIT)
    # The watchdog exits 0 on purpose, so on-failure would leave the view down.
    assert 'Restart=always' in text
    assert 'RestartSec=2' in text
    assert 'StartLimitIntervalSec=300' in text
    assert 'StartLimitBurst=10' in text
    assert 'OnFailure=jcheng-perception-alert@ingress-%i.service' in text
    assert 'PartOf=jcheng-perception.target' in text
    # Stop must address the in-container node, not only the docker exec wrapper.
    assert 'docker exec' in text and 'pkill -TERM' in text
    assert 'lib/jcheng_perception_edge/perception_edge' in text
    assert 'camera_filter:=%i' in text


def test_fusion_restart_policy_and_alert():
    text = unit_text(FUSION_UNIT)
    assert 'Restart=always' in text
    assert 'StartLimitIntervalSec=300' in text
    assert 'StartLimitBurst=10' in text
    assert 'OnFailure=jcheng-perception-alert@fusion.service' in text
    assert 'PartOf=jcheng-perception.target' in text
    assert 'lib/jcheng_perception_edge/perception_fusion' in text


def test_stop_is_not_a_failure():
    """A deliberate stop must not fail the unit or fire OnFailure."""
    for unit in (INGRESS_UNIT, FUSION_UNIT, WRIST_TF_UNIT, EXT_TF_UNIT):
        text = unit_text(unit)
        assert 'Restart=always' in text
        # The wrapper translates the SIGTERM exit only when ExecStop has left
        # the stop marker, so a crash still reaches the start limit.
        assert 'SuccessExitStatus=143' in text, unit
        assert 'SuccessExitStatus=1 ' not in text, unit
        assert '.stopping' in text, unit


def test_stop_marker_translation_in_wrappers():
    for script, marker in (
            (DEPLOY / 'ingress-run.sh', 'ingress-${camera}.stopping'),
            (DEPLOY / 'fusion-run.sh', 'fusion.stopping'),
            (DEPLOY / 'wrist-tf-run.sh', 'wrist-tf.stopping'),
            (DEPLOY / 'ext-tf-run.sh', 'ext-tf.stopping')):
        text = unit_text(script)
        assert 'rm -f "$MARKER"' in text, script
        assert marker in text, script
        assert 'treated as a normal stop' in text, script
        # systemd's TERM must also be reported as a clean stop.
        assert "trap 'STOP_REQUESTED=1' TERM INT" in text, script
        assert '"$STOP_REQUESTED" -eq 1' in text, script
        # No exec: the wrapper must survive docker exec to translate the code.
        assert 'exec docker exec' not in text, script


def test_tf_units_restart_and_alert():
    for unit, label, node in (
            (WRIST_TF_UNIT, 'wrist-tf', 'lib/jcheng_perception_edge/wrist_tf'),
            (EXT_TF_UNIT, 'ext-tf', 'lib/jcheng_perception_edge/ext_tf')):
        text = unit_text(unit)
        assert 'Restart=always' in text
        assert 'StartLimitIntervalSec=300' in text
        assert 'StartLimitBurst=10' in text
        assert 'OnFailure=jcheng-perception-alert@{}.service'.format(
            label) in text
        assert 'PartOf=jcheng-perception.target' in text
        assert node in text
    # wrist_tf needs the vendor message package for the bus-servo service.
    assert 'VENDOR_SETUP' in unit_text(DEPLOY / 'wrist-tf-run.sh')


def test_alert_unit_and_label_mapping():
    text = unit_text(ALERT_UNIT)
    assert 'Type=oneshot' in text
    assert '/usr/local/lib/jcheng-perception/alert.sh %i' in text
    script = unit_text(DEPLOY / 'alert.sh')
    assert 'ingress-*' in script
    for label in ('wrist-tf', 'ext-tf', 'fusion'):
        assert label in script
    assert 'logger -t jcheng-perception -p user.err' in script
    assert 'reset-failed' in script
    assert '/var/lib/jcheng-perception' in script


def test_wrappers_run_one_node_per_unit():
    ingress = unit_text(DEPLOY / 'ingress-run.sh')
    fusion = unit_text(DEPLOY / 'fusion-run.sh')
    wrist_tf = unit_text(DEPLOY / 'wrist-tf-run.sh')
    ext_tf = unit_text(DEPLOY / 'ext-tf-run.sh')
    # A launch file would put three children under one unit again.
    for text in (ingress, fusion, wrist_tf, ext_tf):
        assert 'ros2 launch' not in text
    assert 'exec ros2 run jcheng_perception_edge perception_edge' in ingress
    assert 'exec ros2 run jcheng_perception_edge perception_fusion' in fusion
    assert 'exec ros2 run jcheng_perception_edge wrist_tf' in wrist_tf
    assert 'exec ros2 run jcheng_perception_edge ext_tf' in ext_tf
    for parameter in ('camera_filter', 'publish_targets',
                      'publish_status', 'ingress_watchdog',
                      'cameras_config', 'fusion_config',
                      'frames_config', 'view_bias_config'):
        assert '-p {}:='.format(parameter) in ingress, parameter
    assert 'RMW_IMPLEMENTATION' in ingress and 'CYCLONEDDS_URI' in ingress
    assert 'RMW_IMPLEMENTATION' in fusion and 'CYCLONEDDS_URI' in fusion
    # The camera id becomes part of a quoted container command.
    assert '^[a-z0-9_]+$' in ingress


@pytest.mark.parametrize('script', SCRIPTS, ids=lambda p: p.name)
def test_shell_syntax(script):
    result = subprocess.run(
        ['bash', '-n', str(script)], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr


def _verifiable_copy(tmp_path):
    """Copy the units with paths/users replaced for an off-robot verifier."""
    replacements = {
        'ExecStart=/usr/local/lib/jcheng-perception/ingress-run.sh %i':
            'ExecStart=/bin/true',
        'ExecStart=/usr/local/lib/jcheng-perception/fusion-run.sh':
            'ExecStart=/bin/true',
        'ExecStart=/usr/local/lib/jcheng-perception/wrist-tf-run.sh':
            'ExecStart=/bin/true',
        'ExecStart=/usr/local/lib/jcheng-perception/ext-tf-run.sh':
            'ExecStart=/bin/true',
        'ExecStart=-/usr/local/lib/jcheng-perception/alert.sh %i':
            'ExecStart=/bin/true',
        'User=pi': 'User=root',
        'Group=docker': 'Group=root',
    }
    for unit in (TARGET_UNIT, INGRESS_UNIT, FUSION_UNIT, WRIST_TF_UNIT,
                 EXT_TF_UNIT, ALERT_UNIT):
        text = unit_text(unit)
        for old, new in replacements.items():
            text = text.replace(old, new)
        (tmp_path / unit.name).write_text(text, encoding='utf-8')


@pytest.mark.skipif(
    shutil.which('systemd-analyze') is None,
    reason='systemd-analyze is not installed')
def test_systemd_analyze_verify(tmp_path):
    _verifiable_copy(tmp_path)
    files = [str(tmp_path / unit.name) for unit in (
        TARGET_UNIT, INGRESS_UNIT, FUSION_UNIT, WRIST_TF_UNIT, EXT_TF_UNIT,
        ALERT_UNIT)]
    result = subprocess.run(
        ['systemd-analyze', 'verify', '--man=no', *files],
        capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
