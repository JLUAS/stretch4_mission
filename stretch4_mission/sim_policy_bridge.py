#!/usr/bin/env python3
"""MuJoCo stand-in for policy_bridge.py that moves to the selected pose without running a policy."""

import json
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from stretch4_mission.policy_bridge import policy_pose_file


class SimPolicyBridge(Node):
    def __init__(self):
        super().__init__('stretch4_policy_bridge')
        for name, default in (('task', ''), ('root', ''), ('pose_file', ''), ('ckpt', ''), ('run_args', ''),
                              ('max_runtime_sec', 10.0)):
            self.declare_parameter(name, default)
        self.lift_joint = self.declare_parameter('lift_joint', 'lift_joint').value
        self.arm_joint = self.declare_parameter('arm_joint', 'arm_joint').value
        self.started_at = None
        self.run_sec = 0.0
        group = ReentrantCallbackGroup()
        self.trajectory = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory',
                                       callback_group=group)
        self.create_service(Trigger, '/policy/start', self.start, callback_group=group)
        self.create_service(Trigger, '/policy/stop', self.stop, callback_group=group)
        self.create_service(Trigger, '/policy/status', self.status, callback_group=group)
        self.get_logger().info('SIM bridge ready: /policy/start, /policy/stop, /policy/status')

    @staticmethod
    def wait(future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.05)
        return future.done()

    def running(self):
        return self.started_at is not None and time.monotonic() - self.started_at < self.run_sec

    def start_joints(self):
        path = policy_pose_file(self.get_parameter('root').value, self.get_parameter('pose_file').value)
        with open(path) as f:
            joints = json.load(f)['joints']
        return float(joints['lift']), float(joints['arm'])

    def start(self, request, response):
        if self.running():
            response.success, response.message = False, 'Already running; stop the current policy first.'
            return response
        try:
            lift, arm = self.start_joints()
        except (OSError, ValueError, KeyError) as exc:
            response.success, response.message = False, f'SIM: bad start pose JSON: {exc}'
            return response
        task = self.get_parameter('task').value
        self.get_logger().info(f'SIM start: task={task!r} start_pose lift={lift:.3f} arm={arm:.3f}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.lift_joint, self.arm_joint]
        goal.trajectory.points = [JointTrajectoryPoint(positions=[lift, arm],
                                                       time_from_start=Duration(seconds=5.0).to_msg())]
        if not self.trajectory.wait_for_server(timeout_sec=5.0):
            response.success, response.message = False, 'SIM: trajectory action not available'
            return response
        # Blocking like the real bridge's stretch4_pose goto. Poll async futures (the executor's other
        # threads complete them): the synchronous send_goal() hangs inside a service callback.
        future = self.trajectory.send_goal_async(goal)
        if not self.wait(future, 10.0) or not future.result().accepted:
            response.success, response.message = False, 'SIM: start pose trajectory rejected'
            return response
        self.wait(future.result().get_result_async(), 60.0)
        time.sleep(5.0)  # the sim action returns before the joints move; give it the trajectory time
        self.run_sec = float(self.get_parameter('max_runtime_sec').value)
        self.started_at = time.monotonic()
        response.success = True
        response.message = f'SIM policy START REQUESTED for {task!r} (no policy); not task completion.'
        return response

    def stop(self, request, response):
        self.started_at = None
        response.success, response.message = True, 'SIM: stopped.'
        return response

    def status(self, request, response):
        response.success = True
        if self.running():
            response.message = f'RUNNING; pid=0; elapsed={time.monotonic() - self.started_at:.1f}s; SIM'
        else:
            response.message = 'STOPPED; last_exit_code=0; SIM'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimPolicyBridge()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
