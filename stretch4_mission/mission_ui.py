#!/usr/bin/env python3
"""Web UI for navigation setup, localization, mission editing, and mission execution.

shortcut: no login, anyone who can reach the port can move the robot; add auth before using it off the lab network.
"""

import collections
import json
import math
import os
import signal
import subprocess
import threading
import time
from glob import glob
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from stretch4_mission.mission_file import AUTO, NAME_RE, POSE_RE, load_policies, longest_episode_sec, validate_mission
from stretch4_mission.scan_align import base_points

INDEX_HTML = Path(__file__).with_name('web') / 'index.html'
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
# Start order = list order; stop order = reverse. Each is navigation.launch.py with only that switch true.
COMPONENTS = [('driver', 'Robot driver', 'stretch_driver, navigation mode'),
              ('lidar', 'Lidars', 'dual Hesai, /scan_filtered, footprint'),
              ('nav2', 'Nav2', 'map, AMCL, planner, controller'),
              ('bridge', 'Policy bridge', 'stretch4_policy_recorder: goto, stream, exec, Jetson run')]
# Command-line fragments that identify each part's processes, whoever started them (UI or a terminal).
# 'detect' decides ON/OFF; 'stop' adds companions that should go down with it.
PROCESS_PATTERNS = {
    'driver': {'detect': ('lib/stretch_core/stretch_driver',), 'stop': ('lib/robot_state_publisher/robot_state_publisher',)},
    'lidar': {'detect': ('hesai_ros_driver_node', 'dual_lidar_laserscan', 'robot_footprint_publisher'), 'stop': ()},
    'nav2': {'detect': ('__node:=nav2_container', 'lib/nav2_amcl/amcl', 'lib/nav2_map_server/map_server',
                        'lib/nav2_bt_navigator/bt_navigator', 'lib/nav2_controller/controller_server'), 'stop': ()},
    'bridge': {'detect': ('lib/stretch4_mission/policy_bridge',), 'stop': ()},
}


def scan_processes(patterns=PROCESS_PATTERNS, key='detect'):
    """{component: [pid, ...]} of this user's processes whose command line matches (launch wrappers excluded)."""
    found, me, uid = {name: [] for name in patterns}, os.getpid(), os.getuid()
    for entry in os.listdir('/proc'):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            if os.stat(f'/proc/{entry}').st_uid != uid:
                continue
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if 'ros2 launch' in cmd or 'ros2 run' in cmd:
            continue
        for name, pats in patterns.items():
            fragments = pats['detect'] + (pats['stop'] if key == 'stop' else ())
            if any(fr in cmd for fr in fragments):
                found[name].append(int(entry))
    return found


def wait_future(future, timeout):
    """The executor thread completes futures; HTTP and worker threads just wait."""
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        time.sleep(0.05)
    return future.done()


def sigmas(cov):
    """(xy, yaw) standard deviations from a 6x6 row-major pose covariance."""
    xy_var, yaw_var = max(cov[0], cov[7]), cov[35]
    return tuple(math.sqrt(v) if math.isfinite(v) and v >= 0 else math.inf for v in (xy_var, yaw_var))


class LaunchProcess:
    """One `ros2 launch` child with its own log file; stopped with SIGINT (Ctrl-C), never SIGKILL."""

    def __init__(self, log_dir, kind):
        self.log_dir, self.kind = log_dir, kind
        self.process = self.label = self.log_path = None

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, cmd, label):
        self.log_path = self.log_dir / f"{time.strftime('%Y%m%d-%H%M%S')}_{self.kind}_{label}.log"
        with open(self.log_path, 'wb') as log:  # the child keeps its own handle
            # Ctrl-C must work on the child even if this UI inherited "ignore SIGINT" (e.g. started with `&`).
            self.process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                            start_new_session=True,
                                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
        self.label = label

    def stop(self):
        if self.running():
            os.killpg(self.process.pid, signal.SIGINT)

    def info(self):
        return {'running': self.running(), 'label': self.label,
                'returncode': None if self.process is None else self.process.poll(),
                'log_file': str(self.log_path) if self.log_path else None}

    def tail(self, lines=80):
        if not self.log_path or not self.log_path.exists():
            return []
        with open(self.log_path, 'rb') as f:
            size = f.seek(0, os.SEEK_END)
            f.seek(max(0, size - 16000))
            return f.read().decode(errors='replace').splitlines()[-lines:]


class MissionUI(Node):
    def __init__(self):
        super().__init__('stretch4_mission_ui')
        fleet = os.environ.get('HELLO_FLEET_PATH', '~/stretch_user')

        def param(name, default):
            return self.declare_parameter(name, default).value
        self.missions_dir = Path(os.path.expanduser(param('missions_dir', '~/ament_ws/src/stretch4_mission/missions')))
        self.maps_dir = os.path.expanduser(param('maps_dir', f'{fleet}/maps'))
        self.policies_file = os.path.expanduser(
            param('policies_file', '~/ament_ws/src/stretch4_mission/config/policies.yaml'))
        self.launch_args = list(param('launch_args', ['use_rviz:=true']))
        # 127.0.0.1: reach it through an SSH tunnel (no login on this page); 0.0.0.0 to serve the whole network
        self.host, self.port = param('host', '127.0.0.1'), param('port', 8090)
        self.default_map = os.path.expanduser(param('default_map', ''))
        self.localize_spins = param('localize_spins', 3)
        self.localized_sigma_xy = param('localized_sigma_xy', 0.25)
        self.localized_sigma_yaw = param('localized_sigma_yaw', 0.30)
        self.trajectory_action = param('trajectory_action', '/follow_joint_trajectory')
        log_dir = Path(os.path.expanduser('~/.ros/stretch4_mission'))
        self.missions_dir.mkdir(parents=True, exist_ok=True)
        log_dir.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.procs = {name: LaunchProcess(log_dir, name) for name, _, _ in COMPONENTS}
        self.nav_map = None          # map Nav2 was started with from the UI
        self.external_map = None     # map reported by map_server (Nav2 started from a terminal)
        self.stopping = {}           # component -> note while a stop is in progress
        self.stop_notes = {}         # component -> last stop result worth showing
        self._scan, self._scan_time = {}, 0.0
        self.mission = LaunchProcess(log_dir, 'mission')
        self.seen = {'joints': 0.0, 'scan': 0.0}  # monotonic time of the last message
        self.trace = None
        self.amcl = None         # (stamp_monotonic, sigma_xy, sigma_yaw) of the last /amcl_pose
        self.amcl_count = 0
        self.localizing = False
        self.localize_text = 'Not localized'

        self.create_subscription(String, 'mission/state', self._state_cb, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_cb, LATCHED)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.global_loc = self.create_client(Empty, 'reinitialize_global_localization')
        self.nomotion = self.create_client(Empty, 'request_nomotion_update')
        self.spin_client = ActionClient(self, Spin, 'spin')
        # health checks: graph queries only, nothing is sent
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, self.trajectory_action)
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bridge_client = self.create_client(Trigger, '/policy/status')
        self.map_param_client = self.create_client(GetParameters, '/map_server/get_parameters')
        self.map_param_future = None
        self.create_subscription(JointState, 'joint_states', lambda _: self._seen('joints'), 10)
        self.scans = collections.deque(maxlen=40)
        self.create_subscription(LaserScan, 'scan_filtered', self._scan_cb, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _state_cb(self, msg):
        try:
            self.trace = json.loads(msg.data)
        except ValueError:
            self.trace = {'state': msg.data}

    def _seen(self, key):
        self.seen[key] = time.monotonic()

    def _scan_cb(self, msg):
        self.scans.append(msg)
        self._seen('scan')

    def _amcl_cb(self, msg):
        sxy, syaw = sigmas(msg.pose.covariance)
        self.amcl = (time.monotonic(), sxy, syaw)
        self.amcl_count += 1

    # ---------- files ----------

    def maps(self):
        return sorted(glob(os.path.join(self.maps_dir, '*.yaml')))

    def policies(self):
        return load_policies(self.policies_file)

    @staticmethod
    def policy_start_poses(policy):
        root = Path(os.path.expanduser(policy['root']))
        return sorted(p.name for p in root.glob('*.json') if p.is_file() and POSE_RE.match(p.name))

    @staticmethod
    def policy_runtime(policy):
        """How long the bridge lets this policy run: fixed max_runtime_sec, or longest recording + margin (auto)."""
        if policy['max_runtime_sec'] != AUTO:
            return {'sec': policy['max_runtime_sec'], 'note': 'fixed max_runtime_sec'}
        try:
            longest = longest_episode_sec(policy['root'])
        except ValueError as exc:
            return {'sec': None, 'note': str(exc)}
        return {'sec': longest + policy['runtime_margin_sec'],
                'note': f"longest recording {longest:.1f} s + {policy['runtime_margin_sec']:g} s"}

    def map_meta(self, path):
        if path not in self.maps():
            raise ValueError('unknown map')
        with open(path) as f:
            meta = yaml.safe_load(f)
        image = Path(path).parent / meta['image']
        return meta, image

    def missions(self):
        policies = self.policies()
        out = []
        for path in sorted(self.missions_dir.glob('*.yaml')):
            data = None
            try:
                with open(path) as f:
                    data = yaml.safe_load(f)
                out.append(validate_mission(data, policies))
            except (OSError, yaml.YAMLError, ValueError) as exc:
                entry = {'name': path.stem, 'error': str(exc)}
                if isinstance(data, dict) and isinstance(data.get('steps'), list):
                    # e.g. a policy that no longer exists: send the content so the editor can fix it
                    entry['draft'] = {'name': str(data.get('name') or path.stem), 'map': str(data.get('map', '')),
                                      'steps': [st for st in data['steps'] if isinstance(st, dict)]}
                out.append(entry)
        return out

    def capture(self):
        """Robot pose (map) + reference lidar scan (base_link, median of ~1 s) for a step's goal.

        Park the robot exactly where the policy's episodes started: on arrival the mission aligns the live scan
        to this one, so the step starts from this spot even if AMCL is a few centimeters off.
        """
        pose = self.robot_pose()
        if pose is None:
            raise RuntimeError('No map -> base_link TF: start Nav2 and localize first')
        if time.monotonic() - self.seen['scan'] > 1.0:
            raise RuntimeError('Lidar not ready (no recent /scan_filtered)')
        self.scans.clear()
        time.sleep(1.1)
        scans = list(self.scans)
        if len(scans) < 5:
            raise RuntimeError('Too few lidar scans received; try again')
        try:
            t = self.tf_buffer.lookup_transform('base_link', scans[-1].header.frame_id, Time()).transform
        except TransformException as exc:
            raise RuntimeError(f'No TF {scans[-1].header.frame_id} -> base_link: {exc}') from None
        r = t.rotation
        pts = base_points(scans, (t.translation.x, t.translation.y, euler_from_quaternion([r.x, r.y, r.z, r.w])[2]))
        if len(pts) < 200:
            raise RuntimeError(f'Only {len(pts)} lidar points: too little to align on')
        name = f"scan_{time.strftime('%Y%m%d-%H%M%S')}.json"
        scans_dir = self.missions_dir / 'scans'
        scans_dir.mkdir(exist_ok=True)
        (scans_dir / name).write_text(json.dumps({
            'frame': 'base_link', 'pose_in_map': pose, 'map': self.current_map(),
            'captured_at': time.strftime('%Y-%m-%dT%H:%M:%S'), 'points': np.round(pts, 4).tolist()}))
        return {'pose': pose, 'scan': name, 'points': len(pts)}

    def save_mission(self, body):
        policies = self.policies()
        mission = validate_mission(body, policies, self.maps())
        for i, step in enumerate(mission['steps'], 1):
            if step['start_pose'] not in self.policy_start_poses(policies[step['policy']]):
                raise ValueError(f"step {i}: start pose {step['start_pose']} not found in policy root")
            if not step['scan']:
                continue
            ref = self.missions_dir / 'scans' / step['scan']
            if not ref.is_file():
                raise ValueError(f"step {i}: reference scan {step['scan']} not found")
            captured_on = json.loads(ref.read_text()).get('map')
            # the goal and the reference belong to the map they were captured on; another map puts them elsewhere
            if captured_on and os.path.realpath(captured_on) != os.path.realpath(mission['map']):
                raise ValueError(f"step {i}: its goal and lidar reference were captured on {Path(captured_on).name}; "
                                 f"use that map, or start Nav2 on {Path(mission['map']).name} and capture the goal again")
        path = self.missions_dir / f"{mission['name']}.yaml"
        tmp = path.with_suffix('.tmp')
        tmp.write_text(yaml.safe_dump(mission, sort_keys=False))
        tmp.replace(path)  # atomic: a running mission never reads a half-written file
        return mission['name']

    def delete_mission(self, name):
        if not NAME_RE.match(str(name)) or not (self.missions_dir / f'{name}.yaml').is_file():
            raise ValueError(f'Unknown mission {name!r}')
        if self.mission.running() and self.mission.label == name:
            raise RuntimeError('Mission is running; stop it first')
        (self.missions_dir / f'{name}.yaml').unlink()

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time()).transform
        except TransformException:
            return None
        r = t.rotation
        return {'x': round(t.translation.x, 4), 'y': round(t.translation.y, 4),
                'yaw': round(euler_from_quaternion([r.x, r.y, r.z, r.w])[2], 4)}

    # ---------- robot processes ----------

    def healthy(self, name):
        if name == 'driver':
            return self.trajectory_client.server_is_ready() and time.monotonic() - self.seen['joints'] < 2.0
        if name == 'lidar':
            return time.monotonic() - self.seen['scan'] < 2.0
        if name == 'nav2':
            return self.navigate_client.server_is_ready()
        return self.bridge_client.service_is_ready()

    def processes(self):
        """Process scan, at most once a second (status is polled every second)."""
        if time.monotonic() - self._scan_time > 0.8:
            self._scan, self._scan_time = scan_processes(), time.monotonic()
        return self._scan

    def current_map(self):
        if self.procs['nav2'].running():
            return self.nav_map
        if not self.healthy('nav2'):
            self.external_map = None
            return None
        f = self.map_param_future
        if f is not None and f.done():
            values = f.result().values if f.result() else []
            self.external_map = (values[0].string_value or None) if values else None
            self.map_param_future = None
        if self.external_map is None and self.map_param_future is None and self.map_param_client.service_is_ready():
            self.map_param_future = self.map_param_client.call_async(GetParameters.Request(names=['yaml_filename']))
        return self.external_map

    def component_status(self, name):
        proc, ok, pids = self.procs[name], self.healthy(name), self.processes().get(name, [])
        mine = proc.running()
        on = mine or bool(pids) or ok
        owner = 'ui' if mine else ('terminal' if on else None)
        if name in self.stopping:
            state = self.stopping[name]
        elif on:
            state = ('ready' if ok else 'starting… (not responding yet)') + (' · started from a terminal' if owner == 'terminal' else '')
        else:
            rc = proc.info()['returncode']
            # 0 / -2 / 130: ended normally or by our Ctrl-C
            state = self.stop_notes.get(name) or ('stopped' if rc in (None, 0, -2, 130) else f'exited (code {rc}), see its log')
        return {'name': name, 'on': on, 'ready': ok, 'owner': owner, 'state': state, 'busy': name in self.stopping,
                'running': mine, 'startable': not on and name not in self.stopping,
                'stoppable': on and name not in self.stopping}

    def start_component(self, name, map_path=None):
        with self.lock:
            self._start_component(name, map_path)

    def _start_component(self, name, map_path):
        if name not in self.procs:
            raise ValueError(f'Unknown component {name!r}')
        if name in self.stopping:
            raise RuntimeError(f'{name} is still stopping')
        if self.procs[name].running() or self.processes().get(name) or self.healthy(name):
            # a second driver or Nav2 next to the first would fight over the robot
            raise RuntimeError(f'{name} is already running')
        flags = [f'{c}:={"true" if c == name else "false"}' for c, _, _ in COMPONENTS]
        if name == 'nav2':
            if map_path not in self.maps():
                raise ValueError('choose a map from the list')
            flags.append(f'map:={map_path}')
            self.nav_map = map_path
            self.amcl, self.localize_text = None, 'Not localized: press Localize once Nav2 is ready'
        self.stop_notes.pop(name, None)
        self._scan_time = 0.0
        self.procs[name].start(['ros2', 'launch', 'stretch4_mission', 'navigation.launch.py', *flags,
                                *self.launch_args], name if name != 'nav2' else f'nav2_{Path(map_path).stem}')

    def start_all(self, map_path):
        with self.lock:
            if map_path not in self.maps():
                raise ValueError('choose a map from the list')
            started = []
            for name, _, _ in COMPONENTS:
                if self.component_status(name)['startable']:
                    self._start_component(name, map_path)
                    started.append(name)
            return f"Started: {', '.join(started)}" if started else 'Everything is already running'

    def stop_component(self, name):
        with self.lock:
            if name not in self.procs:
                raise ValueError(f'Unknown component {name!r}')
            if self.mission.running():
                raise RuntimeError('A mission is running; stop it first')
            return self._stop_component(name)

    def _stop_component(self, name):
        st = self.component_status(name)
        if not st['stoppable']:
            return f'{name} is not running' if not st['on'] else f'{name}: {st["state"]}'
        self.stopping[name] = 'stopping… (Ctrl-C sent)'
        self.stop_notes.pop(name, None)
        if self.procs[name].running():
            self.procs[name].stop()           # Ctrl-C to the UI's launch, which stops its nodes
        for pid in scan_processes(key='stop')[name]:
            _signal(pid, signal.SIGINT)       # started from a terminal: Ctrl-C to the nodes themselves
        if name == 'nav2':
            self.amcl, self.localize_text = None, 'Not localized'
        threading.Thread(target=self._finish_stop, args=(name,), daemon=True).start()
        return f'Ctrl-C sent to {name}'

    def _finish_stop(self, name):
        """Ctrl-C, then SIGTERM after 10 s, SIGKILL after 10 s more for whatever is still there."""
        try:
            for sig, note in ((signal.SIGTERM, 'stopping… (SIGTERM)'), (signal.SIGKILL, 'stopping… (SIGKILL)'), (None, None)):
                end = time.monotonic() + 10.0
                while time.monotonic() < end:
                    left = scan_processes(key='stop')[name]
                    if not left and not self.procs[name].running():
                        self.stop_notes[name] = 'stopped' if sig is signal.SIGTERM else 'stopped (had to be forced)'
                        return
                    time.sleep(0.5)
                if sig is None:
                    self.stop_notes[name] = f'did not stop: pids {left}, check its terminal'
                    return
                self.stopping[name] = note
                for pid in left:
                    _signal(pid, sig)
                if self.procs[name].running():
                    _signal(-self.procs[name].process.pid, sig)   # the UI's launch group
        finally:
            self.stopping.pop(name, None)
            self._scan_time = 0.0

    def stop_all(self):
        with self.lock:
            if self.mission.running():
                raise RuntimeError('A mission is running; stop it first')
            stopped = [name for name, _, _ in reversed(COMPONENTS) if self.component_status(name)['stoppable']]
            for name in stopped:
                self._stop_component(name)
            return f"Ctrl-C sent to: {', '.join(stopped)}" if stopped else 'Nothing is running'

    def localized(self):
        a = self.amcl
        return a is not None and a[1] <= self.localized_sigma_xy and a[2] <= self.localized_sigma_yaw

    def localize(self):
        with self.lock:
            if not self.healthy('nav2'):
                raise RuntimeError('Start Nav2 first (and wait until it is ready)')
            if self.mission.running():
                raise RuntimeError('A mission is running')
            if self.localizing:
                raise RuntimeError('Already localizing')
            self.localizing = True
        threading.Thread(target=self._localize, daemon=True).start()

    def _localize(self):
        try:
            self.localize_text = 'Global localization: spreading AMCL particles over the map'
            self._call(self.global_loc, 'reinitialize_global_localization')
            for i in range(self.localize_spins):
                count = self.amcl_count
                for _ in range(5):  # AMCL updates without motion, then the spin gives it new views
                    self._call(self.nomotion, 'request_nomotion_update')
                    time.sleep(0.4)
                if self.amcl_count > count and self.localized():
                    break
                self.localize_text = f'Global localization: spinning in place ({i + 1}/{self.localize_spins})'
                self._spin(2 * math.pi)
            a = self.amcl
            if self.localized():
                self.localize_text = f'Localized (σxy {a[1]:.2f} m, σyaw {a[2]:.2f} rad). Check the robot on the map.'
            else:
                self.localize_text = ('NOT localized: pose estimate did not converge'
                                      + (f' (σxy {a[1]:.2f} m, σyaw {a[2]:.2f} rad)' if a else '')
                                      + '. Move the robot somewhere more distinctive and try again.')
        except RuntimeError as exc:
            self.localize_text = f'Localization failed: {exc}'
        finally:
            self.localizing = False

    def _call(self, client, name):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'AMCL service {name} not available')
        if not wait_future(client.call_async(Empty.Request()), 10.0):
            raise RuntimeError(f'AMCL service {name} did not answer')

    def _spin(self, angle):
        # Nav2's Spin behavior: collision-checked in-place rotation, same server Nav2 uses for recoveries
        if not self.spin_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Nav2 spin action not available')
        goal = Spin.Goal()
        goal.target_yaw = float(angle)
        goal.time_allowance.sec = 60
        future = self.spin_client.send_goal_async(goal)
        if not wait_future(future, 10.0) or not future.result().accepted:
            raise RuntimeError('Spin rejected')
        result = future.result().get_result_async()
        if not wait_future(result, 70.0):
            future.result().cancel_goal_async()
            raise RuntimeError('Spin did not finish')

    def _publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'  # stamp 0 = latest TF
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.02   # sigma ~0.14 m
        msg.pose.covariance[35] = 0.02                            # sigma ~0.14 rad
        count = self.amcl_count
        for _ in range(10):  # resent until AMCL answers (discovery can drop the first one)
            self.initialpose_pub.publish(msg)
            time.sleep(1.0)
            if self.amcl_count > count:
                return
        raise RuntimeError('AMCL did not answer the initial pose')

    # ---------- missions ----------

    def start_mission(self, name):
        with self.lock:
            if self.mission.running():
                raise RuntimeError(f'Mission {self.mission.label} is still running; stop it first')
            if not NAME_RE.match(str(name)) or not (self.missions_dir / f'{name}.yaml').is_file():
                raise ValueError(f'Unknown mission {name!r}')
            with open(self.missions_dir / f'{name}.yaml') as f:
                mission = validate_mission(yaml.safe_load(f), self.policies())
            nav_map = self.current_map()
            if not nav_map:
                raise RuntimeError('Start Nav2 on this mission\'s map first (and wait until it is ready)')
            if os.path.realpath(mission['map']) != os.path.realpath(nav_map):
                raise RuntimeError(f"Mission is for {Path(mission['map']).name}, Nav2 runs {Path(nav_map).name}")
            missing = [label for name, label, _ in COMPONENTS if not self.healthy(name)]
            if missing:
                raise RuntimeError(f"Not ready: {', '.join(missing)}. Start them and wait for 'ready'")
            if self.localizing or not self.localized():
                raise RuntimeError('Localize the robot first')
            self.trace = None
            self.mission.start(['ros2', 'launch', 'stretch4_mission', 'mission.launch.py', 'navigation:=false',
                                f"mission_file:={self.missions_dir / f'{name}.yaml'}",
                                f'policies_file:={self.policies_file}', *self.launch_args], name)

    def stop_mission(self):
        with self.lock:
            if not self.mission.running():
                return 'No mission running'
            # SIGINT = Ctrl-C: the mission cancels its Nav2 goal / arm move or calls /policy/stop.
            self.mission.stop()
            return 'Ctrl-C sent to the mission; the robot processes keep running'

    def status(self, which_log):
        a = self.amcl
        nav = {'running': self.healthy('nav2'), 'map': self.current_map()}
        proc = self.procs.get(which_log, self.mission)
        return {'nav': nav, 'components': [dict(self.component_status(n), label=l, desc=dsc) for n, l, dsc in COMPONENTS],
                'mission': self.mission.info(), 'trace': self.trace, 'pose': self.robot_pose(),
                'localization': {'busy': self.localizing, 'text': self.localize_text,
                                 'localized': self.localized(),
                                 'sigma_xy': a and round(a[1], 3), 'sigma_yaw': a and round(a[2], 3)},
                'log': proc.tail(), 'log_file': proc.info()['log_file'], 'launch_args': self.launch_args}

    def shutdown_children(self):
        for proc in (self.mission, *reversed(list(self.procs.values()))):
            if proc.running():
                self.get_logger().warning(f'UI exiting: sending Ctrl-C to {proc.kind} ({proc.label})')
                proc.stop()


def make_handler(ui):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt, *args):
            pass  # status is polled every second; keep the terminal readable

        def _guard(self, fn):
            try:
                return fn()
            except ValueError as exc:
                self._send(400, {'error': str(exc)})
            except RuntimeError as exc:
                self._send(409, {'error': str(exc)})
            except OSError as exc:
                self._send(500, {'error': str(exc)})

        def do_GET(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}

            def get():
                if url.path == '/':
                    return self._send(200, INDEX_HTML.read_bytes(), 'text/html; charset=utf-8')
                if url.path == '/api/status':
                    return self._send(200, ui.status(q.get('log', 'mission')))
                if url.path == '/api/missions':
                    return self._send(200, ui.missions())
                if url.path == '/api/maps':
                    return self._send(200, ui.maps())
                if url.path == '/api/config':
                    return self._send(200, {'default_map': ui.default_map})
                if url.path == '/api/policies':
                    return self._send(200, {k: {'task': v['task'], 'root': v['root'], 'ckpt': v['ckpt'],
                                                'start_poses': ui.policy_start_poses(v), 'runtime': ui.policy_runtime(v)}
                                            for k, v in ui.policies().items()})
                if url.path == '/api/map_meta':
                    meta, _ = ui.map_meta(q.get('path', ''))
                    keys = ('resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh', 'mode')
                    return self._send(200, {k: meta.get(k) for k in keys})
                if url.path == '/api/map_image':
                    _, image = ui.map_meta(q.get('path', ''))  # validated: only images of listed maps
                    ctype = 'image/png' if image.suffix.lower() == '.png' else 'application/octet-stream'
                    return self._send(200, image.read_bytes(), ctype)
                self._send(404, {'error': 'not found'})
            self._guard(get)

        def do_POST(self):
            def post():
                length = int(self.headers.get('Content-Length', 0))
                if length > 100000:
                    raise ValueError('request too large')
                try:
                    body = json.loads(self.rfile.read(length) or b'{}')
                except ValueError:
                    raise ValueError('expected JSON') from None
                if not isinstance(body, dict):
                    raise ValueError('expected a JSON object')
                routes = {
                    '/api/missions': lambda: {'saved': ui.save_mission(body)},
                    '/api/missions/delete': lambda: (ui.delete_mission(body.get('name')), {'deleted': body.get('name')})[1],
                    '/api/nav/start': lambda: {'message': ui.start_all(body.get('map'))},
                    '/api/nav/stop': lambda: {'message': ui.stop_all()},
                    '/api/component/start': lambda: (ui.start_component(body.get('name'), body.get('map')),
                                                     {'message': f"Starting {body.get('name')}"})[1],
                    '/api/component/stop': lambda: {'message': ui.stop_component(body.get('name'))},
                    '/api/localize': lambda: (ui.localize(), {'message': 'Localizing'})[1],
                    '/api/capture': ui.capture,
                    '/api/start': lambda: (ui.start_mission(body.get('name')), {'started': body.get('name')})[1],
                    '/api/stop': lambda: {'message': ui.stop_mission()},
                }
                if self.path not in routes:
                    return self._send(404, {'error': 'not found'})
                self._send(200, routes[self.path]())
            self._guard(post)

    return Handler


def _signal(pid, sig):
    """Signal a pid (negative: a process group); it may already be gone."""
    try:
        if pid < 0:
            os.killpg(-pid, sig)
        else:
            os.kill(pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # A shell starting us in the background with `&` makes SIGINT ignored, and that would be inherited by every
    # process we launch, so their Stop (Ctrl-C) would do nothing. Restore the normal handler first.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    # SIGTERM (kill, systemd) = Ctrl-C: leave serve_forever() and stop the processes we started. rclpy's own
    # handlers would only shut its context down and leave the web server (and the robot processes) running.
    signal.signal(signal.SIGTERM, _raise_interrupt)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    ui = MissionUI()
    executor = MultiThreadedExecutor()
    executor.add_node(ui)
    threading.Thread(target=executor.spin, daemon=True).start()
    server = ThreadingHTTPServer((ui.host, ui.port), make_handler(ui))
    ui.get_logger().info(f'Mission UI on http://{ui.host}:{ui.port} (missions: {ui.missions_dir}, '
                         f'maps: {ui.maps_dir}, policies: {ui.policies_file}, launch args: {ui.launch_args})')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        ui.shutdown_children()
        server.server_close()
        executor.shutdown()
        ui.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
