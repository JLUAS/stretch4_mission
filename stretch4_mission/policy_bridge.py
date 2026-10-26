#!/usr/bin/env python3
"""Expose stretch4_policy_recorder through /policy/start, /policy/stop, and /policy/status.

Start moves to the selected pose, runs stream/exec on the robot, and runs the policy on the Jetson. Stop shuts
them down in reverse control order. STOPPED describes process state, not task success.
"""

import os
import shlex
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

RUN_PATTERN = 'stretch4_policy_run --ckpt'   # matches the Jetson process for remote stop


def remote_path(path):
    """Quote a path for the remote shell, keeping a leading ~ expandable there (not on the robot)."""
    if path == '~':
        return '"$HOME"'
    if path.startswith('~/'):
        return '"$HOME"/' + shlex.quote(path[2:])
    return shlex.quote(path)


def venv_command(repo, command, remote=False):
    """Shell command: cd into the repo, activate its .venv, exec the tool. `.` instead of `source`: works in sh too."""
    where = remote_path(repo) if remote else shlex.quote(os.path.expanduser(repo))
    return f'cd {where} && . .venv/bin/activate && exec {command}'


def policy_pose_file(root, selected=''):
    """Resolve a selected pose JSON, restricted to the policy's dataset root."""
    root = Path(os.path.expanduser(root)).resolve()
    selected = Path(os.path.expanduser(selected)) if selected else root / 'start_pose.json'
    path = (selected if selected.is_absolute() else root / selected).resolve()
    if path.parent != root:
        raise ValueError(f'pose file must be directly inside {root}')
    return path


def run_command(ckpt, run_args, duration=0.0):
    """stretch4_policy_run with --yes (no terminal to answer "Go?") and --duration so it stops on its own."""
    extra = ['--yes'] + (['--duration', f'{duration:g}'] if duration > 0 else [])
    return ' '.join(['stretch4_policy_run', '--ckpt', remote_path(ckpt), *map(shlex.quote, shlex.split(run_args)), *extra])


class PolicyBridge(Node):
    def __init__(self):
        super().__init__('stretch4_policy_bridge')
        declare = self.declare_parameter
        for name, default in (('task', ''), ('root', ''), ('pose_file', ''), ('ckpt', ''), ('run_args', ''),
                              ('max_runtime_sec', 120.0)):
            declare(name, default)
        self.repo = declare('robot_repo', '~/repos/stretch4_policy_recorder').value
        self.goto_args = declare('goto_extra_args', '').value
        self.goto_timeout = declare('goto_timeout', 120.0).value
        self.jetson_host = declare('jetson_host', 'jetson1@192.168.1.101').value
        self.jetson_repo = declare('jetson_repo', '~/repos/stretch4_policy_recorder').value
        self.ssh_options = list(declare('ssh_options', ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5']).value)
        self.start_delay = declare('start_delay', 3.0).value
        self.stop_timeout = declare('stop_timeout', 10.0).value
        self.log_dir = Path(os.path.expanduser(declare('log_dir', '~/.ros/stretch4_mission').value))
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.procs = {}            # stream / exec / run -> Popen
        self.started_at = None
        self.stopping = False
        self.last_exit = None
        self.log_prefix = None
        group = ReentrantCallbackGroup()
        self.create_service(Trigger, '/policy/start', self.start_cb, callback_group=group)
        self.create_service(Trigger, '/policy/stop', self.stop_cb, callback_group=group)
        self.create_service(Trigger, '/policy/status', self.status_cb, callback_group=group)
        self.create_timer(0.5, self.watch, callback_group=group)
        self.get_logger().info(f'Ready: /policy/start, /policy/stop, /policy/status (Jetson {self.jetson_host})')

    # ---------- processes ----------

    def _spawn(self, name, argv):
        log = open(f'{self.log_prefix}_{name}.log', 'ab')
        # own session: Ctrl-C reaches the tool and its children; Ctrl-C must work even if we inherited "ignore"
        proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
        log.close()
        return proc

    def _ssh(self, command):
        return ['ssh', *self.ssh_options, self.jetson_host, command]

    def _alive(self):
        return {name: proc for name, proc in self.procs.items() if proc.poll() is None}

    def running(self):
        return self.started_at is not None

    # ---------- services ----------

    def start_cb(self, request, response):
        with self.lock:
            response.success, response.message = self._start()
        return response

    def _start(self):
        if self.running():
            return False, 'Already running; stop the current policy first.'
        prm = {n: self.get_parameter(n).value for n in ('task', 'root', 'pose_file', 'ckpt', 'run_args')}
        if not prm['root'] or not prm['ckpt']:
            return False, 'root and ckpt must be set (the mission sets them from policies.yaml)'
        try:
            pose_file = policy_pose_file(prm['root'], prm['pose_file'])
        except ValueError as exc:
            return False, str(exc)
        if not pose_file.is_file():
            return False, f'start pose not found: {pose_file}'
        self.log_prefix = str(self.log_dir / f"policy_{time.strftime('%Y%m%d-%H%M%S')}")
        self.last_exit = None

        goto = venv_command(self.repo, ' '.join(['stretch4_goto_pose', '--root', shlex.quote(os.path.expanduser(prm['root'])),
                                                 '--pose-file', shlex.quote(str(pose_file)), '--yes', self.goto_args]).strip())
        self.get_logger().info(f"{prm['task'] or prm['root']}: going to start pose {pose_file.name}")
        try:
            with open(f'{self.log_prefix}_goto.log', 'ab') as log:
                rc = subprocess.run(['bash', '-c', goto], stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, timeout=self.goto_timeout).returncode
        except subprocess.TimeoutExpired:
            return False, f'stretch4_goto_pose did not finish in {self.goto_timeout:.0f} s; see {self.log_prefix}_goto.log'
        if rc != 0:
            return False, f'stretch4_goto_pose failed (exit {rc}); see {self.log_prefix}_goto.log'

        self.procs = {'stream': self._spawn('stream', ['bash', '-c', venv_command(self.repo, 'stretch4_policy_stream')]),
                      'exec': self._spawn('exec', ['bash', '-c', venv_command(self.repo, 'stretch4_policy_exec')])}
        time.sleep(self.start_delay)
        if len(self._alive()) < 2:
            self._shutdown()
            return False, f'stretch4_policy_stream/exec exited at startup; see {self.log_prefix}_stream/_exec.log'
        run = run_command(prm['ckpt'], prm['run_args'], float(self.get_parameter('max_runtime_sec').value))
        self.procs['run'] = self._spawn('run', self._ssh(venv_command(self.jetson_repo, run, remote=True)))
        time.sleep(2.0)   # ssh refused / tool missing on the Jetson shows up right away
        if self.procs['run'].poll() is not None:
            rc = self.procs['run'].returncode
            self._shutdown()
            return False, f'stretch4_policy_run on {self.jetson_host} exited at once (exit {rc}); see {self.log_prefix}_run.log'
        self.started_at = time.monotonic()
        return True, (f"Policy START REQUESTED: {prm['task'] or prm['root']} (stream, exec, run on {self.jetson_host}); "
                      f'logs {self.log_prefix}_*.log; not task completion.')

    def stop_cb(self, request, response):
        if not self.running():
            response.success, response.message = True, 'No policy running.'
        elif self.stopping:
            response.success, response.message = True, 'Already stopping; check /policy/status.'
        else:
            self.stopping = True
            threading.Thread(target=self._stop_and_clear, daemon=True).start()
            response.success, response.message = True, 'Stopping: Jetson run, then exec, then stream.'
        return response

    def status_cb(self, request, response):
        response.success = True
        if self.running():
            state = 'STOP_REQUESTED' if self.stopping else 'RUNNING'
            alive = ','.join(self._alive())
            response.message = (f'{state}; running={alive}; elapsed={time.monotonic() - self.started_at:.1f}s; '
                                f'logs={self.log_prefix}_*.log; does NOT imply task success')
        else:
            response.message = f'STOPPED; last_exit_code={self.last_exit}; logs={self.log_prefix}_*.log'
        return response

    # ---------- stop ----------

    def watch(self):
        """A process ended by itself, or max_runtime_sec reached: stop the rest."""
        if not self.running() or self.stopping:
            return
        ended = [n for n, p in self.procs.items() if p.poll() is not None]
        limit = float(self.get_parameter('max_runtime_sec').value)
        if ended or (limit > 0 and time.monotonic() - self.started_at >= limit):
            self.get_logger().info(f'{"ended: " + ",".join(ended) if ended else "max runtime reached"}; stopping the rest')
            self.stopping = True
            threading.Thread(target=self._stop_and_clear, daemon=True).start()

    def _stop_and_clear(self):
        try:
            self._shutdown()
        finally:
            run = self.procs.get('run')
            self.last_exit = None if run is None else run.returncode
            self.started_at, self.stopping = None, False

    def _wait(self, names, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and any(self.procs[n].poll() is None for n in names if n in self.procs):
            time.sleep(0.2)

    def _shutdown(self):
        """Jetson first (no more actions), then exec, then stream. Ctrl-C, then SIGTERM."""
        if 'run' in self.procs and self.procs['run'].poll() is None:
            for sig in ('INT', 'TERM'):
                subprocess.run(self._ssh(f'pkill -{sig} -f -- {shlex.quote(RUN_PATTERN)}'), timeout=15,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self._wait(['run'], self.stop_timeout)
                if self.procs['run'].poll() is not None:
                    break
            if self.procs['run'].poll() is None:
                self.procs['run'].terminate()   # the local ssh; the remote may need a manual check
                self.get_logger().error(f'stretch4_policy_run did not stop on {self.jetson_host}; check it there')
        for name in ('exec', 'stream'):
            proc = self.procs.get(name)
            if proc is None or proc.poll() is not None:
                continue
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    break
                self._wait([name], self.stop_timeout)
                if proc.poll() is not None:
                    break
            if proc.poll() is None:
                self.get_logger().error(f'stretch4_policy_{name} still running (pid {proc.pid}); check it')


def main(args=None):
    signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.init(args=args)
    node = PolicyBridge()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        if node.running():
            node.get_logger().warning('Bridge exiting: stopping the running policy')
            node._shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
