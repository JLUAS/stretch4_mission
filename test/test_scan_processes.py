import subprocess
import sys
import time

from stretch4_mission.mission_ui import PROCESS_PATTERNS, scan_processes, sigmas


def sleeper(*argv):
    # argv after the script lands in /proc/<pid>/cmdline, like the real nodes' command lines
    return subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)', *argv])


def test_scan_finds_parts_by_command_line_and_skips_launch_wrappers():
    bridge = sleeper('/home/x/tec_workspace/install/stretch4_mission/lib/stretch4_mission/policy_bridge')
    wrapper = sleeper('ros2 run', 'stretch4_mission', 'lib/stretch4_mission/policy_bridge')   # a wrapper, not the node
    rsp = sleeper('/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher')
    try:
        time.sleep(0.3)
        found = scan_processes()
        assert bridge.pid in found['bridge'] and wrapper.pid not in found['bridge']
        assert rsp.pid not in found['driver']                  # a companion alone does not mean "driver on"
        assert rsp.pid in scan_processes(key='stop')['driver']  # ...but it is stopped together with the driver
    finally:
        for p in (bridge, wrapper, rsp):
            p.kill()


def test_every_component_has_patterns():
    assert set(PROCESS_PATTERNS) == {'driver', 'lidar', 'nav2', 'bridge'}
    assert all(p['detect'] for p in PROCESS_PATTERNS.values())


def test_invalid_covariance_is_not_localized():
    cov = [0.0] * 36
    cov[0], cov[7], cov[35] = 0.04, 0.09, -1.0
    assert sigmas(cov) == (0.3, float('inf'))
