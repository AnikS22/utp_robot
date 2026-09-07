import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'bringup'))
from stop_localizer import is_localizer


def command(*args):
    return b'\0'.join(x.encode() for x in args) + b'\0'


def test_only_exact_localizer_and_launcher_match():
    assert is_localizer(command('/opt/ros/jazzy/lib/slam_toolbox/localization_slam_toolbox_node', '--ros-args'))
    assert is_localizer(command('/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'run', 'slam_toolbox', 'localization_slam_toolbox_node'))
    assert not is_localizer(command('bash', '-c', 'ros2 run slam_toolbox localization_slam_toolbox_node'))
    assert not is_localizer(command('python3', 'scan_relay.py', 'localization_slam_toolbox_node'))
    assert not is_localizer(command('/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node'))
