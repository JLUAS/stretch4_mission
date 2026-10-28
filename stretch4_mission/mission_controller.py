#!/usr/bin/env python3
"""Run a mission's steps in order: policy, navigate-only, and command steps.

Execution is sequential so Nav2, a policy and a command never control the robot together. Progress is published as JSON on
/mission/state for the web UI.
"""

import collections
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint

from stretch4_mission.mission_file import bridge_params, load_mission, load_policies, parse_goal
from stretch4_mission.scan_align import base_points, correction, icp

PHASES = ('RETRACTING_ARM', 'CHECKING_POSE', 'NAVIGATING', 'ALIGNING', 'PREPARING_POLICY', 'EXECUTING_POLICY',
          'RUNNING_COMMAND')
# what each step type goes through; every type starts with the arm retract
STEP_PHASES = {'policy': PHASES[:6], 'navigate': PHASES[:4], 'command': ('RETRACTING_ARM', 'RUNNING_COMMAND')}
LOG_DIR = Path(os.path.expanduser('~/.ros/stretch4_mission'))


class MissionFailed(Exception):
    pass


def pose_error(x, y, yaw, tx, ty, tyaw):
    """Return (distance, signed yaw to rotate to reach the target, wrapped to [-pi, pi])."""
    dyaw = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
    return math.hypot(x - tx, y - ty), dyaw


class MissionTrace:
    """Mission progress as one JSON-able dict: overall state plus a status per step and phase.

    Phase status: pending | active | done | skipped | failed.
    """

    def __init__(self, name, steps):
        self.data = {'mission': name, 'state': 'INITIALIZING', 'step': None, 'detail': '',
                     'steps': [{'type': s.get('type', 'policy'), 'policy': s.get('policy'), 'config': s.get('config'),
                                'goal': s.get('goal'), 'command': s.get('command'), 'result': '',
                                'phases': {p: 'pending' for p in STEP_PHASES[s.get('type', 'policy')]}} for s in steps]}

    def _finish_active(self, status):
        for step in self.data['steps']:
            for phase, value in step['phases'].items():
                if value == 'active':
                    step['phases'][phase] = status

    def enter(self, state, step=None, detail=''):
        self._finish_active('done')
        self.data.update(state=state, step=step, detail=detail)
        if step is not None and state in PHASES:
            self.data['steps'][step]['phases'][state] = 'active'

    def skip(self, step, phase):
        self.data['steps'][step]['phases'][phase] = 'skipped'

    def result(self, step, text):
        self.data['steps'][step]['result'] = text

    def fail(self, detail):
        self._finish_active('failed')
        self.data.update(state='FAILED', detail=detail)

    def complete(self, detail):
        self._finish_active('done')
        self.data.update(state='COMPLETED', step=None, detail=detail)


class MissionController(BasicNavigator):
    PARAMS = {
        'mission_file': '',
        'goal': '',                  # "x,y,yaw": go to this pose instead of running mission_file (UI "Go to pose")
        'policies_file': '~/ament_ws/src/stretch4_mission/config/policies.yaml',
        'global_frame': 'map',
        'robot_base_frame': 'base_link',
        'odom_topic': '/wheel_odom',
        'xy_tolerance': 0.25,
        'yaw_tolerance': 0.15,
        'nav2_goal_checker': 'general_goal_checker',
        'nav2_xy_goal_tolerance': 0.10,
        'nav2_yaw_goal_tolerance': 0.08,
        'set_initial_pose': False,
        'initial_x': 0.0,
        'initial_y': 0.0,
        'initial_yaw': 0.0,
        'stationary_linear_speed': 0.01,
        'stationary_angular_speed': 0.02,
        'stationary_duration': 1.0,
        # final alignment by lidar scan matching (steps with a reference scan)
        'scan_topic': '/scan_filtered',
        'cmd_vel_topic': '/cmd_vel',
        'align_position_tolerance': 0.01,
        'align_yaw_tolerance': 0.01,
        'align_max_offset': 0.40,
        'align_max_yaw_offset': 0.50,
        'align_min_inliers': 0.60,
        'align_max_iterations': 6,
        'align_speed': 0.05,
        'align_yaw_speed': 0.15,
        'align_min_speed': 0.01,
        'align_min_yaw_speed': 0.03,
        'align_clearance': 0.26,
        'align_move_timeout': 20.0,
        'trajectory_action': '/follow_joint_trajectory',
        'joint_states_topic': '/joint_states',
        'retract_arm_before_navigation': True,
        'lift_joint': 'lift_joint',
        'lift_retract_position': 0.5,
        'lift_retract_tolerance': 0.01,
        'lift_retract_duration': 6.0,
        'arm_joint': 'arm_joint',
        'arm_state_joints': ['arm_l1_joint', 'arm_l2_joint', 'arm_l3_joint', 'arm_l4_joint'],
        'arm_retracted_max': 0.02,
        'arm_retract_duration': 5.0,
        'wrist_yaw_joint': 'wrist_yaw_joint',
        'wrist_roll_joint': 'wrist_roll_joint',
        'wrist_pitch_joint': 'wrist_pitch_joint',
        'wrist_yaw_stow_position': 3.14,
        'wrist_roll_stow_position': 0.0,
        'wrist_pitch_stow_position': 0.0,
        'wrist_stow_tolerance': 0.03,
        'wrist_stow_duration': 4.0,
        'lift_before_policy': True,
        'lift_up_position': 1.2,
        'lift_min_height': 1.16,
        'lift_up_duration': 10.0,
        'bridge_node': '/stretch4_policy_bridge',
        'start_service': '/policy/start',
        'stop_service': '/policy/stop',
        'status_service': '/policy/status',
        'nav2_ready_timeout': 120.0,
        'localization_timeout': 120.0,
        'navigation_timeout': 300.0,
        'stationary_timeout': 20.0,
        'arm_timeout': 30.0,
        'lift_timeout': 30.0,
        'policy_start_timeout': 200.0,
        'policy_timeout': 300.0,
        'status_period': 2.0,
        'command_timeout': 300.0,    # s, command steps without their own timeout_sec
    }

    def __init__(self):
        super().__init__(node_name='stretch4_mission_controller')
        self.p = {k: self.declare_parameter(k, v).value for k, v in self.PARAMS.items()}
        self.trace = None
        self.state = None
        self.state_pub = self.create_publisher(
            String, 'mission/state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom = None
        self.odom_stamp = 0.0
        self.create_subscription(Odometry, self.p['odom_topic'], self._odom_cb, 10)
        self.joints = {}
        self.create_subscription(JointState, self.p['joint_states_topic'], self._joints_cb, 10)
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, self.p['trajectory_action'])
        self.scans = collections.deque(maxlen=40)
        self.create_subscription(LaserScan, self.p['scan_topic'], lambda m: self.scans.append(m), qos_profile_sensor_data)
        self.cmd_vel = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        self.trajectory_goal = None
        self.command = None   # the running command step's process
        self.clients_ = {}

    # ---------- helpers ----------

    def _odom_cb(self, msg):
        self.odom = msg
        self.odom_stamp = time.monotonic()

    def _joints_cb(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def set_state(self, state, step=None, detail=''):
        self.state = state
        if state == 'FAILED':
            self.trace.fail(detail)
        elif state == 'COMPLETED':
            self.trace.complete(detail)
        else:
            self.trace.enter(state, step, detail)
        where = f' [step {step + 1}/{len(self.trace.data["steps"])}]' if step is not None else ''
        text = f'{state}{where}: {detail}' if detail else f'{state}{where}'
        (self.error if state == 'FAILED' else self.info)(f'[MISSION] {text}')
        self.publish_trace()

    def publish_trace(self):
        self.state_pub.publish(String(data=json.dumps(self.trace.data)))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, end - time.monotonic())))

    def call_trigger(self, name, timeout):
        client = self.clients_.get(name) or self.clients_.setdefault(name, self.create_client(Trigger, name))
        if not client.wait_for_service(timeout_sec=min(timeout, 10.0)):
            raise MissionFailed(f'Service {name} not available')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise MissionFailed(f'Service {name} gave no response within {timeout:.0f}s')
        return future.result()

    def set_remote_params(self, node, params):
        client = self.create_client(SetParameters, f'{node}/set_parameters')
        if not client.wait_for_service(timeout_sec=10.0):
            raise MissionFailed(f'{node}/set_parameters not available')
        future = client.call_async(SetParameters.Request(
            parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in params.items()]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        results = future.result().results if future.result() is not None else []
        failed = [f'{k}: {r.reason}' for k, r in zip(params, results) if not r.successful]
        if len(results) != len(params) or failed:
            raise MissionFailed(f'Could not set parameters on {node}: {failed or "no response"}')

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.p['global_frame']
        # stamp 0 requests the latest available TF
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
        pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
        return pose

    def current_pose(self, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    self.p['global_frame'], self.p['robot_base_frame'], Time()).transform
            except TransformException:
                continue
            r = t.rotation
            return t.translation.x, t.translation.y, euler_from_quaternion([r.x, r.y, r.z, r.w])[2]
        raise MissionFailed(f"No TF {self.p['global_frame']} -> {self.p['robot_base_frame']}")

    def at_target(self, goal):
        x, y, yaw = self.current_pose()
        dist, dyaw = pose_error(x, y, yaw, goal['x'], goal['y'], goal['yaw'])
        self.info(f'Pose x={x:.3f} y={y:.3f} yaw={yaw:.3f} | error dist={dist:.3f} m yaw={dyaw:.3f} rad')
        return dist <= self.p['xy_tolerance'] and abs(dyaw) <= self.p['yaw_tolerance']

    def wait_task(self, what, timeout):
        deadline = time.monotonic() + timeout
        while not self.isTaskComplete():
            if time.monotonic() > deadline:
                self.cancelTask()
                raise MissionFailed(f'{what} timed out; goal canceled')
        result = self.getResult()
        if result != TaskResult.SUCCEEDED:
            raise MissionFailed(f'{what} ended with {result.name}')

    # ---------- mission steps ----------

    def wait_lifecycle_active(self, node_name, deadline):
        client = self.create_client(GetState, f'{node_name}/get_state')
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() is not None and future.result().current_state.label == 'active':
                    return
            self.info(f'Waiting for {node_name} to become active...')
            self.spin_for(2.0)
        raise MissionFailed(f'{node_name} not active within timeout')

    def wait_nav2_and_localization(self):
        # Order matters: the navigation servers (global costmap) only finish activating once the
        # map frame exists, which needs AMCL to have a pose; so localize before waiting for bt_navigator.
        deadline = time.monotonic() + self.p['nav2_ready_timeout']
        self.wait_lifecycle_active('amcl', deadline)
        self.wait_localization()
        self.wait_lifecycle_active('bt_navigator', time.monotonic() + self.p['nav2_ready_timeout'])

    def wait_localization(self):
        # BasicNavigator.waitUntilNav2Active() is not used: it republishes a zero initial pose
        # until AMCL answers, which would relocalize the robot at the map origin.
        # amcl_pose is latched, so a robot already localized from the UI passes at once.
        deadline = time.monotonic() + self.p['localization_timeout']
        if self.p['set_initial_pose']:
            initial = (self.p['initial_x'], self.p['initial_y'], self.p['initial_yaw'])
            self.info(f'Publishing initial pose x={initial[0]} y={initial[1]} yaw={initial[2]}')
        else:
            self.info('Waiting for AMCL pose: localize from the UI or RViz "2D Pose Estimate"')
        while not self.initial_pose_received:
            if time.monotonic() > deadline:
                raise MissionFailed('AMCL did not publish a pose (robot not localized)')
            if self.p['set_initial_pose']:
                self.setInitialPose(self.make_pose(*initial))  # resent until AMCL answers
            self.spin_for(2.0)

    def set_nav2_goal_tolerance(self):
        # stretch_nav2 accepts ~0.34 rad heading error. Tighten its SimpleGoalChecker at runtime
        # (dynamic params; resets when Nav2 restarts) so MPPI does the final alignment smoothly.
        # Nav2 Spin was tried for this and overshot ~0.23 rad on Stretch 4.
        checker = self.p['nav2_goal_checker']
        self.set_remote_params('controller_server', {
            f'{checker}.xy_goal_tolerance': float(self.p['nav2_xy_goal_tolerance']),
            f'{checker}.yaw_goal_tolerance': float(self.p['nav2_yaw_goal_tolerance'])})

    def navigate(self, goal):
        self.set_nav2_goal_tolerance()
        if not self.goToPose(self.make_pose(goal['x'], goal['y'], goal['yaw'])):
            raise MissionFailed('NavigateToPose goal rejected')
        self.wait_task('Navigation', self.p['navigation_timeout'])

    def wait_stationary(self):
        deadline = time.monotonic() + self.p['stationary_timeout']
        still_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            fresh = self.odom is not None and time.monotonic() - self.odom_stamp < 0.5
            if fresh:
                tw = self.odom.twist.twist
                still = (math.hypot(tw.linear.x, tw.linear.y) <= self.p['stationary_linear_speed']
                         and abs(tw.angular.z) <= self.p['stationary_angular_speed'])
            else:
                still = False
            if not still:
                still_since = None
            elif still_since is None:
                still_since = time.monotonic()
            elif time.monotonic() - still_since >= self.p['stationary_duration']:
                return
        raise MissionFailed(f"Base not stationary (or no {self.p['odom_topic']}) within timeout")

    def settled(self, measure, wait=0.3):
        """measure() if it holds still over `wait` seconds, else None."""
        a = measure()
        self.spin_for(wait)
        b = measure()
        return b if a is not None and b is not None and abs(a - b) < 0.002 else None

    def move_joint(self, what, joint, target, duration, timeout, measure, ok):
        """One-joint FollowJointTrajectory goal, judged only on the measured joint (measure() satisfying ok(), settled).

        Not on the action result: stretch_driver only finishes a goal after the joint has *started* moving
        (stretch_core command_groups is_finished), so a goal at the current position (arm already retracted,
        lift already at its clipped max) never finishes until the driver's action_timeout. So: skip when already
        there, succeed as soon as the joint is,
        cancel whatever the driver is still tracking, and fail at once only if the driver aborts with an error.
        """
        value = self.settled(measure)
        if value is not None and ok(value):
            self.info(f'{what}: already there ({value:.3f}), nothing to move')
            return
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            raise MissionFailed(f"{self.p['trajectory_action']} not available")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint]
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[float(target)], time_from_start=Duration(seconds=duration).to_msg())]
        self.info(f'{what}: {joint} -> {target:.3f}')
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise MissionFailed(f'{what}: trajectory rejected by {self.p["trajectory_action"]}')
        self.trajectory_goal = handle
        result = handle.get_result_async()
        try:
            deadline = time.monotonic() + timeout
            previous = None
            while time.monotonic() < deadline:
                self.spin_for(0.5)
                value = measure()
                if value is not None and previous is not None and ok(value) and abs(value - previous) < 0.002:
                    self.info(f'{what}: done at {value:.3f}')
                    return
                previous = value
                if result.done() and result.result() is not None and result.result().result.error_code != 0:
                    r = result.result().result
                    raise MissionFailed(f'{what}: driver aborted the trajectory ({r.error_code}: {r.error_string}); '
                                        f'measured {value}')
            raise MissionFailed(f'{what}: measured {value}, not at an acceptable value and settled within {timeout:.0f} s')
        finally:
            if not result.done():
                handle.cancel_goal_async()   # the driver may still be "tracking" a goal that is already reached
            self.trajectory_goal = None

    # ---------- final alignment (lidar scan matching) ----------

    def live_points(self, seconds=1.0):
        """base_link points from the median of the scans received during `seconds` (robot standing still)."""
        self.scans.clear()
        self.spin_for(seconds)
        if len(self.scans) < 3:
            raise MissionFailed(f"No lidar scans on {self.p['scan_topic']}")
        last = self.scans[-1]
        try:
            t = self.tf_buffer.lookup_transform(self.p['robot_base_frame'], last.header.frame_id, Time()).transform
        except TransformException as exc:
            raise MissionFailed(f'No TF {last.header.frame_id} -> {self.p["robot_base_frame"]}: {exc}') from None
        r = t.rotation
        return base_points(list(self.scans), (t.translation.x, t.translation.y,
                                              euler_from_quaternion([r.x, r.y, r.z, r.w])[2]))

    def odom_pose(self):
        if self.odom is None or time.monotonic() - self.odom_stamp > 0.5:
            raise MissionFailed(f"No fresh odometry on {self.p['odom_topic']}")
        q = self.odom.pose.pose.orientation
        return self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, \
            euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def stop_base(self):
        for _ in range(5):
            self.cmd_vel.publish(Twist())
            time.sleep(0.02)

    def clearance(self):
        """Closest lidar point to the base center (m), from the latest scan."""
        last = self.scans[-1] if self.scans else None
        if last is None:
            return float('inf')
        r = [x for x in last.ranges if math.isfinite(x) and x > last.range_min]
        return min(r) if r else float('inf')

    def move_relative(self, dx, dy, dyaw):
        """Slow closed-loop base move by (dx, dy, dyaw) in the robot frame, on odometry (omni base: x, y, yaw)."""
        x0, y0, yaw0 = self.odom_pose()
        tx = x0 + math.cos(yaw0) * dx - math.sin(yaw0) * dy
        ty = y0 + math.sin(yaw0) * dx + math.cos(yaw0) * dy
        tyaw = yaw0 + dyaw
        deadline = time.monotonic() + self.p['align_move_timeout']

        def speed(err, tol, vmin, vmax):
            # proportional, but never below vmin while outside tolerance: the base does not move at a few mm/s
            if abs(err) <= tol:
                return 0.0
            return math.copysign(min(vmax, max(vmin, 1.5 * abs(err))), err)
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.05)
                x, y, yaw = self.odom_pose()
                ex_w, ey_w = tx - x, ty - y
                ex = math.cos(yaw) * ex_w + math.sin(yaw) * ey_w
                ey = -math.sin(yaw) * ex_w + math.cos(yaw) * ey_w
                eyaw = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
                if abs(ex) <= 0.002 and abs(ey) <= 0.002 and abs(eyaw) <= 0.003:
                    return
                if time.monotonic() > deadline:
                    raise MissionFailed('Alignment move timed out')
                if self.clearance() < self.p['align_clearance']:
                    raise MissionFailed(f'Alignment stopped: obstacle {self.clearance():.2f} m from the base center')
                cmd = Twist()
                cmd.linear.x = speed(ex, 0.002, self.p['align_min_speed'], self.p['align_speed'])
                cmd.linear.y = speed(ey, 0.002, self.p['align_min_speed'], self.p['align_speed'])
                cmd.angular.z = speed(eyaw, 0.003, self.p['align_min_yaw_speed'], self.p['align_yaw_speed'])
                self.cmd_vel.publish(cmd)
        finally:
            self.stop_base()

    def align_to_reference(self, scan_file, goal):
        """Match the live scan to the step's reference scan and move until they overlay (the recorded spot)."""
        path = Path(os.path.expanduser(self.p['mission_file'])).parent / 'scans' / scan_file
        try:
            ref = np.array(json.loads(path.read_text())['points'], dtype=float)
        except (OSError, ValueError, KeyError) as exc:
            raise MissionFailed(f'Reference scan {scan_file}: {exc}') from None
        x, y, yaw = self.current_pose()
        dist, dyaw = pose_error(x, y, yaw, goal['x'], goal['y'], goal['yaw'])
        if dist > self.p['align_max_offset'] or abs(dyaw) > self.p['align_max_yaw_offset']:
            raise MissionFailed(f'AMCL puts the robot {dist:.2f} m / {dyaw:.2f} rad from the goal: too far to align')
        for i in range(int(self.p['align_max_iterations'])):
            est, inliers, rms = icp(self.live_points(), ref)
            dx, dy, dth = correction(est)
            off = math.hypot(dx, dy)
            self.info(f'Align {i + 1}: offset {off * 100:.1f} cm, {math.degrees(dth):.2f} deg '
                      f'(match {inliers:.0%}, rms {rms * 100:.1f} cm)')
            self.trace.data['detail'] = f'scan match: offset {off * 100:.1f} cm, {math.degrees(dth):.2f} deg, {inliers:.0%} match'
            self.publish_trace()
            if inliers < self.p['align_min_inliers']:
                raise MissionFailed(f'Live scan does not match the reference ({inliers:.0%} of points): '
                                    'the surroundings changed or the robot is not near the recorded spot')
            if off <= self.p['align_position_tolerance'] and abs(dth) <= self.p['align_yaw_tolerance']:
                self.info(f'Aligned: {off * 100:.1f} cm, {math.degrees(dth):.2f} deg from the recorded spot')
                return
            if off > self.p['align_max_offset'] or abs(dth) > self.p['align_max_yaw_offset']:
                raise MissionFailed(f'Scan match asks for {off:.2f} m / {dth:.2f} rad: too large, not moving')
            self.move_relative(dx, dy, dth)
            self.wait_stationary()
        raise MissionFailed(f"Alignment did not converge in {int(self.p['align_max_iterations'])} moves")

    def retract_arm(self):
        # Below travel height: lift to it first, then retract. At or above it: retract first, then lower to it.
        # The wrist is stowed last either way.
        lift, target = self.joints.get(self.p['lift_joint']), self.p['lift_retract_position']
        if lift is None:
            raise MissionFailed(f"No {self.p['lift_joint']} position in joint_states; arm not moved")

        def to_travel_height():
            self.move_joint('Lift to travel height', self.p['lift_joint'], target, self.p['lift_retract_duration'],
                            self.p['lift_timeout'], lambda: self.joints.get(self.p['lift_joint']),
                            lambda v: abs(v - target) <= self.p['lift_retract_tolerance'])

        def retract():
            def extension():
                values = [self.joints.get(j) for j in self.p['arm_state_joints']]
                return None if None in values else sum(values)
            self.move_joint('Retract arm', self.p['arm_joint'], 0.0, self.p['arm_retract_duration'],
                            self.p['arm_timeout'], extension, lambda v: v <= self.p['arm_retracted_max'])

        for move in ((to_travel_height, retract) if lift < target else (retract, to_travel_height)):
            move()

        # Match the installed SG4 stow orientation without commanding its gripper.
        for axis in ('yaw', 'roll', 'pitch'):
            joint = self.p[f'wrist_{axis}_joint']
            target = self.p[f'wrist_{axis}_stow_position']
            self.move_joint(f'Stow wrist {axis}', joint, target, self.p['wrist_stow_duration'],
                            self.p['arm_timeout'], lambda joint=joint: self.joints.get(joint),
                            lambda v, target=target: abs(v - target) <= self.p['wrist_stow_tolerance'])

    def lift_up(self):
        # Raise only the lift before the bridge's start-pose move (stretch4_goto_pose or stretch4_pose goto), which
        # may interpolate all joints without collision checking; coming down from the top keeps the arm clear.
        self.move_joint('Lift up', self.p['lift_joint'], self.p['lift_up_position'], self.p['lift_up_duration'],
                        self.p['lift_timeout'], lambda: self.joints.get(self.p['lift_joint']),
                        lambda v: v >= self.p['lift_min_height'])

    def run_policy(self, step_index, policy_name, params):
        status = self.call_trigger(self.p['status_service'], 10.0)
        if not status.message.startswith('STOPPED'):
            raise MissionFailed(f'Policy client already active: {status.message}')
        # The bridge reads these at every start; set them only while no policy runs.
        self.set_remote_params(self.p['bridge_node'], params)
        self.set_state('EXECUTING_POLICY', step_index,
                       f'{policy_name}: calling start (pose {Path(params["pose_file"]).name})')
        # The bridge moves to the start pose inside this call, then starts the runner's processes.
        res = self.call_trigger(self.p['start_service'], self.p['policy_start_timeout'])
        if not res.success:
            raise MissionFailed(f"{self.p['start_service']} refused: {res.message}")
        self.info(res.message)

        deadline = time.monotonic() + self.p['policy_timeout']
        last = None
        while True:
            self.spin_for(self.p['status_period'])
            msg = self.call_trigger(self.p['status_service'], 10.0).message
            state = msg.split(';', 1)[0]
            if state != last:
                last = state
                self.info(f'Policy status: {msg}')
                self.trace.data['detail'] = f'{policy_name}: {msg}'
                self.publish_trace()
            if msg.startswith('STOPPED'):
                return msg
            if time.monotonic() > deadline:
                self.call_trigger(self.p['stop_service'], 10.0)
                raise MissionFailed('Policy timeout; stop requested, wait for STOPPED before moving robot')

    def run(self):
        if self.p['goal']:
            return self.run_goto()
        try:
            policies = load_policies(os.path.expanduser(self.p['policies_file']))
            mission = load_mission(os.path.expanduser(self.p['mission_file']), policies)
            # before anything moves: "auto" runtimes read the datasets, and the controller must outwait the bridge
            step_params = {}   # step index -> bridge parameters, policy steps only
            for i, step in enumerate(mission['steps']):
                if step['type'] != 'policy':
                    continue
                root = Path(os.path.expanduser(policies[step['policy']]['root'])).resolve()
                pose_file = (root / step['start_pose']).resolve()
                if pose_file.parent != root or not pose_file.is_file():
                    raise ValueError(f'step {i + 1}: start pose not found in policy root: {step["start_pose"]}')
                prm = step_params[i] = bridge_params(policies[step['policy']], pose_file, step.get('max_runtime_sec'),
                                                     step.get('run_args'))
                if prm['max_runtime_sec'] >= self.p['policy_timeout']:
                    raise ValueError(f"step {i + 1}: max_runtime_sec {prm['max_runtime_sec']:.1f} s must be below "
                                     f"policy_timeout {self.p['policy_timeout']:.0f} s (config/mission.yaml)")
                self.info(f"step {i + 1}: {step['policy']}: max runtime {prm['max_runtime_sec']:.1f} s")
        except (OSError, ValueError) as exc:
            self.trace = MissionTrace(os.path.basename(self.p['mission_file']), [])
            self.set_state('FAILED', detail=f'Bad mission/policies file: {exc}')
            return False
        self.trace = MissionTrace(mission['name'], mission['steps'])
        for i, prm in step_params.items():
            self.trace.data['steps'][i]['max_runtime_sec'] = prm['max_runtime_sec']   # the limit this run really uses
        try:
            self.set_state('INITIALIZING', detail=f"{mission['name']}: {len(mission['steps'])} step(s)")
            self.wait_nav2_and_localization()

            for i, step in enumerate(mission['steps']):
                kind = step['type']
                name = step['policy'] if kind == 'policy' else kind
                if self.p['retract_arm_before_navigation']:
                    self.set_state('RETRACTING_ARM', i, name)
                    self.retract_arm()
                else:
                    self.trace.skip(i, 'RETRACTING_ARM')
                if kind == 'command':
                    self.trace.result(i, self.run_command(i, step))
                    continue

                goal = step['goal']

                self.set_state('CHECKING_POSE', i, name)
                if self.at_target(goal):
                    self.info('Already within tolerance; skipping navigation')
                    self.trace.skip(i, 'NAVIGATING')
                else:
                    self.set_state('NAVIGATING', i, f"{name}: goal x={goal['x']:.2f} y={goal['y']:.2f} "
                                                    f"yaw={goal['yaw']:.2f}")
                    self.navigate(goal)

                self.set_state('ALIGNING', i, name)
                self.wait_stationary()
                if step['scan']:
                    self.align_to_reference(step['scan'], goal)
                elif not self.at_target(goal):
                    raise MissionFailed('Stopped outside mission tolerance' + ('; policy not started' if kind == 'policy' else ''))
                if kind == 'navigate':
                    continue

                self.set_state('PREPARING_POLICY', i, name)
                if self.p['lift_before_policy']:
                    self.lift_up()

                result = self.run_policy(i, name, step_params[i])
                # Not task success: only that the policy process ended.
                self.trace.result(i, result)
                photos = dict(kv.split('=', 1) for kv in result.split('; ') if '=' in kv).get('photos')
                if photos:   # the bridge's pictures after the policy (recorder runner), for the Operator view
                    self.trace.data['steps'][i]['photos'] = Path(photos).name

            self.set_state('COMPLETED', detail='all steps finished. '
                                               'Verify the tasks physically; this is not task success.')
            return True
        except MissionFailed as exc:
            self.set_state('FAILED', detail=str(exc))
            return False

    def run_goto(self):
        """Go to pose with stretch_nav_webapp's backend: its stow (stretch4_body), then its NavigateToPose client.

        No alignment, lift or policy.
        """
        try:
            goal = parse_goal(self.p['goal'])
            from stretch4_nav_webapp import nav_actions, robot_client_helper   # installed with stretch_nav_webapp
        except (ValueError, ImportError) as exc:
            self.trace = MissionTrace('Go to pose', [])
            self.set_state('FAILED', detail=str(exc))
            return False
        self.trace = MissionTrace('Go to pose', [{'type': 'navigate', 'goal': goal}])
        try:
            self.set_state('INITIALIZING', detail='Go to pose')
            self.wait_nav2_and_localization()
            self.set_state('RETRACTING_ARM', 0, 'stow (stretch_nav_webapp)')
            try:
                robot_client_helper.cmd_stow()   # the robot's own stow pose; refuses when not homed
            except RuntimeError as exc:
                raise MissionFailed(f'Stow failed: {exc}') from None
            self.set_state('CHECKING_POSE', 0, 'go to pose')
            if self.at_target(goal):
                self.info('Already within tolerance; skipping navigation')
                self.trace.skip(0, 'NAVIGATING')
            else:
                self.set_state('NAVIGATING', 0, f"goal x={goal['x']:.2f} y={goal['y']:.2f} yaw={goal['yaw']:.2f}")
                self.webapp_navigate(nav_actions, goal)
            self.trace.skip(0, 'ALIGNING')
            self.set_state('COMPLETED', detail=f"at x={goal['x']:.2f} y={goal['y']:.2f} yaw={goal['yaw']:.2f}")
            return True
        except MissionFailed as exc:
            self.set_state('FAILED', detail=str(exc))
            return False

    def run_command(self, step_index, step):
        """Command step: its shell line with bash in the home folder, ROS environment inherited; exit 0 = success."""
        timeout = step.get('timeout_sec', self.p['command_timeout'])
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        log = LOG_DIR / f"command_{time.strftime('%Y%m%d-%H%M%S')}.log"
        self.set_state('RUNNING_COMMAND', step_index, f"$ {step['command']}  (log {log})")
        with open(log, 'wb') as f:   # own session: stop_command ends the whole tree, scripts it started too
            self.command = subprocess.Popen(['bash', '-c', step['command']], cwd=os.path.expanduser('~'), stdout=f,
                                            stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        deadline = time.monotonic() + timeout
        while self.command.poll() is None:
            if time.monotonic() > deadline:
                self.stop_command()
                raise MissionFailed(f'Command still running after {timeout:.0f} s; stopped (log {log})')
            self.spin_for(0.5)
        code, self.command = self.command.returncode, None
        if code != 0:
            raise MissionFailed(f'Command exited with code {code} (log {log})')
        self.info(f'Command finished (exit code 0, log {log})')
        return f'exit code 0; log {log}'

    def stop_command(self, grace=5.0):
        """Ctrl-C to the command's process group, up to grace seconds to exit, then SIGKILL to whatever is left."""
        proc, self.command = self.command, None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(grace)
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass
        try:   # always: a non-interactive bash starts its background children with SIGINT ignored
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(2.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            self.error(f'Command process {proc.pid} did not stop; check it')

    def webapp_navigate(self, nav_actions, goal):
        nav_actions.send_navigate_to_pose(goal['x'], goal['y'], goal['yaw'])
        # shortcut: the webapp returns once Nav2 accepts and keeps the goal handle to itself; waiting on it is the
        # only way to know the result. Upgrade if stretch_nav_webapp gets a "wait for result" helper.
        result = nav_actions._goal_handle.get_result_async()
        deadline = time.monotonic() + self.p['navigation_timeout']
        while not result.done():
            if time.monotonic() > deadline:
                nav_actions.cancel_navigate_to_pose()
                raise MissionFailed('Navigation timed out; goal canceled')
            nav_actions._spin_once(0.2)
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise MissionFailed(f'Navigation ended with status {status} (4 = succeeded, 5 = canceled, 6 = aborted)')

    def abort(self):
        """Best effort on Ctrl-C: release whichever controller this mission started."""
        try:
            if self.state == 'NAVIGATING' and self.p['goal']:   # Go to pose: the goal was sent by the webapp client
                from stretch4_nav_webapp import nav_actions
                nav_actions.cancel_navigate_to_pose()
            elif self.state == 'NAVIGATING':
                self.cancelTask()
            elif self.state == 'ALIGNING':
                self.stop_base()
            elif self.state == 'RUNNING_COMMAND':
                self.stop_command()
            elif self.trajectory_goal is not None:
                self.trajectory_goal.cancel_goal_async()
            elif self.state == 'EXECUTING_POLICY':
                self.call_trigger(self.p['stop_service'], 5.0)
        except Exception as exc:  # noqa: BLE001 - shutting down, report and continue
            self.error(f'Abort cleanup failed: {exc}')
        if self.trace is not None:
            self.set_state('FAILED', detail='interrupted by user')


def main(args=None):
    # No rclpy SIGINT handler: keep the context alive so abort() can still cancel/stop.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionController()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.abort()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
