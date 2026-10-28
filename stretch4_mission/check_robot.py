#!/usr/bin/env python3
"""Is this robot ready for the mission UI? Read-only checks, each with how to fix it.

    ros2 run stretch4_mission check_robot              # everything
    ros2 run stretch4_mission check_robot --no-jetson  # skip the ssh checks on the Jetson

Exit code 0 when nothing failed (warnings allowed), 1 otherwise. It starts, stops and moves nothing.
"""

import argparse
import json
import os
import socket
import subprocess
import sys
from glob import glob
from pathlib import Path

import yaml

from stretch4_mission.mission_file import AUTO, RUNTIMES_FILE, load_policies, recorded_runtime, validate_mission
from stretch4_mission.policy_bridge import remote_path

OK, WARN, FAIL = 'ok', 'warn', 'fail'
MARK = {OK: '✓', WARN: '!', FAIL: '✗'}


def running(pattern):
    """Command lines of processes matching a pgrep -f pattern (this checker excluded)."""
    out = subprocess.run(['pgrep', '-af', pattern], capture_output=True, text=True).stdout.splitlines()
    return [line for line in out if 'check_robot' not in line]


def check_environment(src):
    yield (OK if os.environ.get('ROS_DISTRO') == 'jazzy' else FAIL,
           f"ROS_DISTRO={os.environ.get('ROS_DISTRO')}", 'source /opt/ros/jazzy/setup.bash and the workspace')
    rmw = os.environ.get('RMW_IMPLEMENTATION')
    yield (OK if rmw == 'rmw_zenoh_cpp' else WARN, f'RMW_IMPLEMENTATION={rmw}',
           'export RMW_IMPLEMENTATION=rmw_zenoh_cpp (scripts/run_ui_robot.sh does)')
    yield (OK if running('rmw_zenohd') else WARN, 'zenoh router (rmw_zenohd)',
           'without it ROS nodes do not find each other; check the zenohd service')
    fleet = os.environ.get('HELLO_FLEET_PATH')
    yield (OK if fleet and os.path.isdir(fleet) else FAIL, f'HELLO_FLEET_PATH={fleet}',
           'export HELLO_FLEET_PATH=~/stretch_user (scripts/run_ui_robot.sh does)')
    yield (OK if (src / 'config' / 'policies.yaml').is_file() else FAIL, f'package source: {src}',
           'pass --src <folder with config/, missions/> (where the repo was cloned)')


def check_robot():
    with socket.socket() as s:
        s.settimeout(1.0)
        busy = s.connect_ex(('127.0.0.1', 8090)) == 0
    yield (FAIL if busy else OK, 'UI port 8090 ' + ('already in use' if busy else 'free'),
           'the UI is already running: use it, or stop it with pkill -TERM -f lib/stretch4_mission/mission_ui')
    yield (OK if running('stretch_body_server') else FAIL, 'stretch_body_server',
           'the driver, stow and policies need it: stretch_body_server --launch')
    stray = running(r'stretch4_policy_(stream|exec|run)')
    yield (WARN if stray else OK, 'no stretch4_policy_stream/exec running by hand' if not stray else
           f'already running: {"; ".join(line[:80] for line in stray)}',
           'the bridge starts its own; stop these first or its start fails (ports 5580/5581 in use)')
    try:   # stretch_nav_webapp's own readiness probe (the one its page polls), when installed
        out = subprocess.run([sys.executable, '-c', 'import json; from stretch4_nav_webapp.robot_client_helper '
                              'import cmd_readiness; print(json.dumps(cmd_readiness()))'],
                             capture_output=True, text=True, timeout=20)
        ready = json.loads(out.stdout.strip().splitlines()[-1]) if out.returncode == 0 else None
    except (subprocess.TimeoutExpired, IndexError, ValueError):
        ready = None
    if isinstance(ready, dict) and 'homed' in ready:
        yield (OK if ready['homed'] else FAIL, 'robot homed' if ready['homed'] else 'robot NOT homed',
               'run stretch_robot_home before starting the driver')
        if ready.get('runstop_engaged'):
            yield WARN, f"runstop engaged ({ready.get('runstop_cause')})", 'release it before a mission'
        soc = ready.get('battery_soc')
        if soc is not None:
            yield (WARN if soc < 20 else OK), f'battery {soc:.0f} %', 'charge the robot'
    else:
        yield WARN, 'homing not checked (stretch_nav_webapp not installed or robot server not answering)', \
            'make sure the robot is homed: stretch_robot_home'


def check_maps(maps_dir, default_map):
    maps = sorted(glob(os.path.join(maps_dir, '*.yaml')) + glob(os.path.join(maps_dir, '*', 'map.yaml')))
    yield (OK if maps else FAIL, f'{len(maps)} map(s) in {maps_dir}', 'save a map there (map_saver_cli or stretch_nav_webapp)')
    for path in maps:
        try:
            with open(path) as f:
                image = Path(path).parent / yaml.safe_load(f)['image']
            if not image.is_file():
                yield FAIL, f'map {path}: image {image} missing', 'copy the .pgm next to its .yaml'
        except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
            yield FAIL, f'map {path}: unreadable ({exc})', 'fix or remove that file'
    yield (OK if os.path.isfile(default_map) else WARN, f'default map {default_map}',
           'scripts/run_ui_robot.sh preselects it; choose another map in the UI or change the script')


def check_policies(policies_file):
    try:
        policies = load_policies(policies_file)
    except (OSError, ValueError) as exc:
        yield FAIL, f'policies: {exc}', f'fix {policies_file}'
        return {}
    yield OK, f'{len(policies)} polic(ies) in {policies_file}', ''
    for name, p in policies.items():
        root = Path(os.path.expanduser(p['root']))
        if not root.is_dir():
            yield FAIL, f'{name}: dataset {root} missing', 'copy the dataset to the robot, or fix root in policies.yaml'
            continue
        if not (root / 'start_pose.json').is_file():
            yield FAIL, f'{name}: {root}/start_pose.json missing', 'the start pose comes from the dataset'
        if p['runner'] == 'lerobot' and not (Path(os.path.expanduser(p['dataset'])) / 'meta').is_dir():
            yield FAIL, f"{name}: LeRobot dataset {p['dataset']} missing", 'copy it to the robot, or fix dataset in policies.yaml'
        if p['max_runtime_sec'] == AUTO:
            try:
                sec, source = recorded_runtime(p)
                partial = source.startswith('only')
                yield (WARN if partial else OK, f"{name}: run time {sec + p['runtime_margin_sec']:.1f} s (auto; {source})",
                       f'copy the dataset again, or {RUNTIMES_FILE} from the robot that recorded it' if partial else '')
            except ValueError as exc:
                yield FAIL, f'{name}: {exc}', (f'copy {RUNTIMES_FILE} from the robot that recorded it, '
                                               'or set a fixed max_runtime_sec in policies.yaml')
        else:
            yield OK, f"{name}: run time {p['max_runtime_sec']:g} s (fixed)", ''
    return policies


def check_missions(missions_dir, policies, maps):
    if not any(missions_dir.glob('*.yaml')):
        yield OK, f'no missions yet in {missions_dir}', ''
    for path in sorted(missions_dir.glob('*.yaml')):
        try:
            with open(path) as f:
                mission = validate_mission(yaml.safe_load(f), policies, maps)
            missing = [s['scan'] for s in mission['steps'] if s.get('scan') and not (missions_dir / 'scans' / s['scan']).is_file()]
            yield ((FAIL, f'mission {path.stem}: lidar references missing: {", ".join(missing)}',
                    'capture the goals again in the editor') if missing else (OK, f'mission {path.stem}', ''))
        except (OSError, ValueError, yaml.YAMLError) as exc:
            yield FAIL, f'mission {path.stem}: {exc}', 'open it in the editor (Edit) and save it again'


def check_runner(runner_file, policies, jetson):
    try:
        with open(runner_file) as f:
            prm = yaml.safe_load(f)['stretch4_policy_bridge']['ros__parameters']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        yield FAIL, f'policy runner settings: {exc}', f'fix {runner_file}'
        return
    runners = {p['runner'] for p in policies.values()}
    tools = []
    if 'recorder' in runners:
        tools += [(prm['robot_repo'], t, 'install stretch4_policy_recorder with its .venv (robot_repo in policy_runner.yaml)')
                  for t in ('stretch4_goto_pose', 'stretch4_policy_stream', 'stretch4_policy_exec')]
    if 'lerobot' in runners:
        tools += [(prm.get('lerobot_robot_repo', '~/repos/lerobot_stretch4'), t,
                   'install lerobot_stretch4 with its .venv (lerobot_robot_repo in policy_runner.yaml)')
                  for t in ('stretch4_pose', 'python')]
    for repo, tool, fix in tools:
        path = Path(os.path.expanduser(repo)) / '.venv' / 'bin' / tool
        yield OK if path.is_file() else FAIL, f'robot: {path}', fix
    if not jetson:
        return
    host = prm['jetson_host']
    ssh = ['ssh', *prm.get('ssh_options', []), host]
    try:
        up = subprocess.run([*ssh, 'true'], capture_output=True, text=True, timeout=15).returncode == 0
    except subprocess.TimeoutExpired:
        up = False
    yield (OK if up else FAIL, f'ssh to the Jetson {host}',
           f'ssh -o BatchMode=yes {host} true must work without a password (ssh-copy-id {host})')
    if not up:
        return
    files = []   # (test flag, path, what): -f a file, -d a folder
    if 'recorder' in runners:
        files.append(('-f', f"{remote_path(prm['jetson_repo'])}/.venv/bin/stretch4_policy_run", 'stretch4_policy_run'))
    if 'lerobot' in runners:
        repo = remote_path(prm.get('lerobot_jetson_repo', '~/repos/lerobot_stretch4'))
        files.append(('-f', f'{repo}/.venv/bin/stretch4_policy_server', 'stretch4_policy_server'))
    files += [('-d' if p['runner'] == 'lerobot' else '-f', remote_path(p['ckpt']), f"{name}: checkpoint {p['ckpt']}")
              for name, p in policies.items()]
    script = '; '.join(f'test {flag} {path} && echo 1 || echo 0' for flag, path, _ in files)
    flags = subprocess.run([*ssh, script], capture_output=True, text=True, timeout=15).stdout.split()
    for (_, _, what), flag in zip(files, flags + ['0'] * len(files)):
        yield (OK if flag == '1' else FAIL, f'Jetson: {what}', 'copy it to the Jetson (or fix the path)')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--src', default='~/ament_ws/src/stretch4_mission',
                   help='the package source (config/, missions/); e.g. ~/tec_workspace/stretch4_mission')
    p.add_argument('--maps', default=None, help='maps folder (default $HELLO_FLEET_PATH/maps)')
    p.add_argument('--no-jetson', action='store_true', help='skip the ssh checks on the Jetson')
    args = p.parse_args()
    src = Path(os.path.expanduser(args.src))
    maps_dir = args.maps or os.path.join(os.environ.get('HELLO_FLEET_PATH', '~/stretch_user'), 'maps')
    maps_dir = os.path.expanduser(maps_dir)

    failed = 0

    def report(title, results):
        nonlocal failed
        print(f'\n{title}')
        for level, text, fix in results:
            failed += level == FAIL
            print(f'  {MARK[level]} {text}' + (f'\n      → {fix}' if fix and level != OK else ''))

    report('Environment', check_environment(src))
    report('Robot', check_robot())
    report('Maps', check_maps(maps_dir, os.path.join(maps_dir, 'test.yaml')))
    policies = {}

    def collect():   # keep check_policies' return value
        nonlocal policies
        policies = yield from check_policies(src / 'config' / 'policies.yaml')
    report('Policies', collect())
    maps = sorted(glob(os.path.join(maps_dir, '*.yaml')) + glob(os.path.join(maps_dir, '*', 'map.yaml')))
    report('Missions', check_missions(src / 'missions', policies, maps))
    report('Policy runner', check_runner(src / 'config' / 'policy_runner.yaml', policies, not args.no_jetson))
    print(f"\n{'READY' if not failed else f'NOT READY: {failed} problem(s) above'}")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
