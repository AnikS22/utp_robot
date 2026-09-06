"""Offline startup regressions: deadlines, discovery, process scope and CLI errors."""
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'bringup'))
from startup_wait import wait_for_transforms
from startup_processes import matching_processes
from stop_stack import owned_processes


def test_missing_transforms_share_one_deadline():
    now = [0.0]
    def spin():
        now[0] += .25
    result = wait_for_transforms([('map', 'odom'), ('base', 'lidar'), ('a', 'b')],
                                 lambda *_: False, spin, 3, lambda: now[0])
    assert not any(result.values())
    assert now[0] == 3


def test_late_transform_is_discovered_while_another_edge_is_missing():
    now = [0.0]
    def spin():
        now[0] += .25
    result = wait_for_transforms([('map', 'odom'), ('base', 'lidar')],
                                 lambda a, b: a == 'base' and now[0] >= 1,
                                 spin, 2, lambda: now[0])
    assert result == {('map', 'odom'): False, ('base', 'lidar'): True}


def process(proc, pid, parent, command, repo='/robot', domain='9'):
    p = proc / str(pid)
    p.mkdir()
    (p / 'stat').write_text(f'{pid} (name with spaces) S {parent} 0 0')
    (p / 'cmdline').write_bytes(command)
    (p / 'environ').write_bytes(f'UTP_ROBOT_STACK={repo}\0ROS_DOMAIN_ID={domain}\0'.encode())


def test_process_matching_excludes_callers_and_empty_patterns(tmp_path):
    process(tmp_path, 10, 1, b'bash\0robot\0')
    process(tmp_path, 11, 10, b'python\0robot\0')
    process(tmp_path, 12, 1, b'robot\0driver\0')
    assert list(matching_processes('robot', tmp_path, pid=11)) == [12]
    with pytest.raises(ValueError):
        list(matching_processes('', tmp_path, pid=11))


def test_shutdown_selects_only_this_checkout_and_domain(tmp_path):
    process(tmp_path, 100, 1, b'python\0driver\0')
    process(tmp_path, 101, 1, b'python\0driver\0', domain='42')
    process(tmp_path, 102, 1, b'python\0driver\0', repo='/simulation')
    assert [pid for pid, _ in owned_processes(Path('/robot'), '9', tmp_path)] == [100]


@pytest.mark.parametrize('args', [['--mode'], ['--map'], ['--mode', 'invalid'], ['--map', '../bad']])
def test_invalid_arguments_exit_instead_of_hanging(args):
    result = subprocess.run(['bash', str(ROOT / 'bringup/bringup_all.sh'), *args],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 2


def shell(body):
    return subprocess.run(['bash', '-c', f'source "{ROOT}/bringup/startup_helpers.sh"\n' + body],
                          capture_output=True, text=True, timeout=5)


def test_readiness_retries_then_exits_as_soon_as_data_arrives():
    result = shell('''
      declare -A P; attempts=0
      note() { :; }; sleep() { :; }; ge() { [ "$1" -ge "$2" ]; }
      hz() { echo "$attempts"; }
      probe() { attempts=$((attempts+1)); }
      wait_ready 2 /scan 2 topic:/scan:sensor_msgs.msg:LaserScan:reliable
      echo "$attempts"
    ''')
    assert result.returncode == 0
    assert result.stdout.strip() == '2'


def test_ready_topic_does_not_hide_missing_tf():
    result = shell('''
      declare -A P; attempts=0
      note() { :; }; sleep() { SECONDS=$((SECONDS+1)); }; ge() { return 0; }
      hz() { echo 10; }; tfok() { return 1; }
      probe() { attempts=$((attempts+1)); }
      wait_ready 2 /scan 2 topic:/scan:sensor_msgs.msg:LaserScan:reliable tf:map:odom
    ''')
    assert result.returncode == 1


def test_active_lifecycle_node_is_not_reconfigured():
    result = shell('''
      ros2() { if [ "$2" = get ]; then echo 'active [3]'; else return 99; fi; }
      timeout() { shift; "$@"; }
      LOG=/dev/null
      ensure_active /slam_toolbox 2
    ''')
    assert result.returncode == 0


def test_map_provenance_refuses_wrong_live_map(tmp_path):
    from startup_mark_map import record_session
    (tmp_path / 'maps').mkdir()
    for ext in ('pgm', 'yaml', 'posegraph', 'data'):
        (tmp_path / 'maps' / ('floor1.' + ext)).write_bytes(b'saved')
    marker = tmp_path / 'maps/.loaded_map'
    marker.write_text('previous session\n')
    with pytest.raises(ValueError):
        record_session(tmp_path, 'floor1', 'mapping', str(tmp_path / 'maps/floor1'), 'abc')
    assert marker.read_text() == 'previous session\n'
    with pytest.raises(ValueError):
        record_session(tmp_path, 'floor1', 'localization', str(tmp_path / 'maps/floor2'), 'abc')
    record_session(tmp_path, 'floor1', 'localization', str(tmp_path / 'maps/floor1'), 'abc')
    assert marker.read_text() == 'floor1 abc\n'


def test_inactive_lifecycle_node_is_not_reported_active():
    result = shell("""
      ros2() { if [ "$2" = get ]; then echo 'inactive [2]'; else return 1; fi; }
      timeout() { shift; "$@"; }
      LOG=/dev/null
      ensure_active /slam_toolbox 2
    """)
    assert result.returncode == 1


def test_inputs_mode_never_enters_slam_startup():
    script = (ROOT / 'bringup/bringup_all.sh').read_text()
    a = script.index('if [ "$MODE" != inputs ]; then')
    b = script.index('# Named waypoint commands', a)
    result = subprocess.run(['bash', '-c', 'MODE=inputs\nrecord() { echo "$*"; }\n' + script[a:b]],
                            capture_output=True, text=True, timeout=3)
    assert result.returncode == 0
    assert result.stdout.strip() == 'slam skip inputs-only: no mapping or localization'


@pytest.mark.parametrize('state,frames,expected', [
    ('ERROR-ACTIVE', 100, True), ('ERROR-PASSIVE', 100, False),
    ('ERROR-ACTIVE', 0, False), ('BUS-OFF', 0, False)])
def test_can_requires_real_feedback_and_healthy_bus(state, frames, expected):
    from check_inputs import can_health
    link = {'operstate': 'UP', 'linkinfo': {'info_data': {'state': state}}}
    assert can_health(link, frames)['ok'] is expected


def test_image_audit_rejects_truncated_frames():
    from types import SimpleNamespace
    from check_inputs import image_health
    assert image_health(SimpleNamespace(width=2, height=2, step=6, data=bytes(12)))
    assert not image_health(SimpleNamespace(width=2, height=2, step=6, data=bytes(11)))


def test_inputs_wrapper_rejects_unknown_actions_before_startup():
    result = subprocess.run(['bash', str(ROOT / 'bringup/inputs.sh'), 'drive'],
                            capture_output=True, text=True, timeout=3)
    assert result.returncode == 2
    assert 'usage:' in result.stderr


def test_arm_tool_reads_reported_values_not_initial_sdk_zeros():
    from arm_tool import read_tool_state
    class Arm:
        tcp_offset = [0] * 6
        tcp_load = [0, [0, 0, 0]]
        released = False
        def register_report_callback(self, callback):
            self.tcp_offset = [0, 0, 172, 0, 0, 0]
            self.tcp_load = [.82, [0, 0, 48]]
            callback({})
        def release_report_callback(self, callback):
            self.released = True
    arm = Arm()
    offset, load = read_tool_state(arm)
    assert offset[2] == 172 and load[0] == .82 and arm.released


def test_arm_tool_refuses_silent_report_stream():
    from arm_tool import read_tool_state
    class Arm:
        released = False
        def register_report_callback(self, callback): pass
        def release_report_callback(self, callback): self.released = True
    arm = Arm()
    with pytest.raises(TimeoutError):
        read_tool_state(arm, timeout=0)
    assert arm.released
