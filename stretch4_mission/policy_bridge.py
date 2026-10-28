#!/usr/bin/env python3
"""Run a policy behind /policy/start, /policy/stop, and /policy/status, with one of two runners.

recorder (stretch4_policy_recorder): stretch4_goto_pose, stream/exec on the robot, stretch4_policy_run on the Jetson.
lerobot (lerobot_stretch4): stretch4_pose goto, stretch4_policy_server on the Jetson (unless one already listens),
LeRobot's async robot_client on the robot. Stop shuts them down in STOP_ORDER. STOPPED describes process state, not
task success.
"""

import json
import os
import shlex
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

# processes on the Jetson, stopped there with pkill (the local ssh only carries their output)
REMOTE_PATTERNS = {'run': 'stretch4_policy_run --ckpt', 'server': 'stretch4_policy_server --host'}
STOP_ORDER = ('run', 'client', 'exec', 'stream', 'server')   # whatever sends actions first, then the rest
MAIN = {'recorder': 'run', 'lerobot': 'client'}   # its exit code is the run's last_exit_code
SNAPSHOT = Path(__file__).with_name('snapshot.py')   # run with the recorder's Python: it decodes the stream


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


def lerobot_client_command(dataset, task, server, ckpt, policy_type, run_args):
    """LeRobot's async robot_client; everything after --policy_type comes from the policy's run_args."""
    return ' '.join(['python', '-m', 'lerobot.async_inference.robot_client', '--robot.type=stretch4', '--robot.mode=control',
                     shlex.quote(f'--robot.preprocessing_from={os.path.expanduser(dataset)}'),
                     shlex.quote(f'--task={task}'), shlex.quote(f'--server_address={server}'),
                     shlex.quote(f'--pretrained_name_or_path={ckpt}'), shlex.quote(f'--policy_type={policy_type}'),
                     *map(shlex.quote, shlex.split(run_args))])


def port_open(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


class PolicyBridge(Node):
    def __init__(self):
        super().__init__('stretch4_policy_bridge')
        declare = self.declare_parameter
        for name, default in (('task', ''), ('root', ''), ('pose_file', ''), ('ckpt', ''), ('run_args', ''),
                              ('max_runtime_sec', 120.0), ('runner', 'recorder'), ('dataset', ''), ('policy_type', '')):
            declare(name, default)
        self.repo = declare('robot_repo', '~/repos/stretch4_policy_recorder').value
        self.goto_args = declare('goto_extra_args', '').value
        self.goto_timeout = declare('goto_timeout', 120.0).value
        self.jetson_host = declare('jetson_host', 'jetson1@192.168.1.101').value
        self.jetson_repo = declare('jetson_repo', '~/repos/stretch4_policy_recorder').value
        self.ssh_options = list(declare('ssh_options', ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5']).value)
        self.lerobot_repo = declare('lerobot_robot_repo', '~/repos/lerobot_stretch4').value
        self.lerobot_jetson_repo = declare('lerobot_jetson_repo', '~/repos/lerobot_stretch4').value
        self.server_port = declare('lerobot_server_port', 8080).value
        self.server_timeout = declare('lerobot_server_timeout', 60.0).value
        # recorder policies: the picture after the policy, from these stream cameras ([] = none); see snapshot.py
        self.snapshot_cameras = list(declare('snapshot_cameras', ['head_right', 'gripper']).value)
        self.start_delay = declare('start_delay', 3.0).value
        self.stop_timeout = declare('stop_timeout', 10.0).value
        self.log_dir = Path(os.path.expanduser(declare('log_dir', '~/.ros/stretch4_mission').value))
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.procs = {}            # stream / exec / run (recorder), client / server (lerobot) -> Popen
        self.main = 'run'          # the process whose exit code is reported
        self.started_at = None
        self.stopping = False
        self.last_exit = None
        self.log_prefix = None
        self.photos = None         # folder with the pictures taken when the last policy stopped
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
        prm = {n: self.get_parameter(n).value
               for n in ('task', 'root', 'pose_file', 'ckpt', 'run_args', 'runner', 'dataset', 'policy_type')}
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
        self.photos = None
        self.main = MAIN.get(prm['runner'], 'run')
        if prm['runner'] == 'lerobot':
            return self._start_lerobot(prm, pose_file)

        goto = venv_command(self.repo, ' '.join(['stretch4_goto_pose', '--root', shlex.quote(os.path.expanduser(prm['root'])),
                                                 '--pose-file', shlex.quote(str(pose_file)), '--yes', self.goto_args]).strip())
        self.get_logger().info(f"{prm['task'] or prm['root']}: going to start pose {pose_file.name}")
        error = self._goto(goto, 'stretch4_goto_pose')
        if error:
            return False, error

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

    def _goto(self, command, tool):
        """Run a start-pose move to the end; -> an error message, or None."""
        try:
            with open(f'{self.log_prefix}_goto.log', 'ab') as log:
                rc = subprocess.run(['bash', '-c', command], stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, timeout=self.goto_timeout).returncode
        except subprocess.TimeoutExpired:
            return f'{tool} did not finish in {self.goto_timeout:.0f} s; see {self.log_prefix}_goto.log'
        return f'{tool} failed (exit {rc}); see {self.log_prefix}_goto.log' if rc != 0 else None

    def _start_lerobot(self, prm, pose_file):
        dataset = Path(os.path.expanduser(prm['dataset']))
        if not (dataset / 'meta').is_dir():
            return False, f'LeRobot dataset not found on the robot: {dataset} (no meta/)'
        # the server loads the model by this path: absolute, as the Jetson sees it, and checked before anything moves
        try:
            out = subprocess.run(self._ssh(f"cd {remote_path(prm['ckpt'])} && pwd"), capture_output=True, text=True,
                                 timeout=20)
        except subprocess.TimeoutExpired:
            return False, f'no answer from {self.jetson_host} in 20 s'
        if out.returncode != 0:
            return False, f"model {prm['ckpt']} not found on {self.jetson_host}: {out.stderr.strip()}"
        ckpt = out.stdout.strip()

        self.get_logger().info(f"{prm['task'] or prm['root']}: going to start pose {pose_file.name} (stretch4_pose)")
        error = self._goto(venv_command(self.lerobot_repo, f'stretch4_pose goto {shlex.quote(str(pose_file))} --yes'),
                           'stretch4_pose goto')
        if error:
            return False, error

        host = self.jetson_host.split('@')[-1]
        self.procs = {}
        if port_open(host, self.server_port):
            self.get_logger().info(f'stretch4_policy_server already listens on {host}:{self.server_port}; using it')
        else:   # ours: started now, stopped with the policy
            server = f'stretch4_policy_server --host=0.0.0.0 --port={int(self.server_port)}'
            self.procs['server'] = self._spawn('server', self._ssh(venv_command(self.lerobot_jetson_repo, server, remote=True)))
            end = time.monotonic() + self.server_timeout
            while not port_open(host, self.server_port):
                if self.procs['server'].poll() is not None or time.monotonic() > end:
                    self._shutdown()
                    return False, (f'stretch4_policy_server on {self.jetson_host} did not listen on port '
                                   f'{self.server_port}; see {self.log_prefix}_server.log')
                time.sleep(0.5)
        client = lerobot_client_command(prm['dataset'], prm['task'], f'{host}:{self.server_port}', ckpt,
                                        prm['policy_type'], prm['run_args'])
        self.procs['client'] = self._spawn('client', ['bash', '-c', venv_command(self.lerobot_repo, client)])
        time.sleep(self.start_delay)   # a bad flag or no server shows up right away
        if self.procs['client'].poll() is not None:
            rc = self.procs['client'].returncode
            self._shutdown()
            return False, f'robot_client exited at once (exit {rc}); see {self.log_prefix}_client.log'
        self.started_at = time.monotonic()
        return True, (f"Policy START REQUESTED: {prm['task'] or prm['root']} (LeRobot {prm['policy_type']}, server "
                      f'{host}:{self.server_port}); logs {self.log_prefix}_*.log; not task completion.')

    def stop_cb(self, request, response):
        if not self.running():
            response.success, response.message = True, 'No policy running.'
        elif self.stopping:
            response.success, response.message = True, 'Already stopping; check /policy/status.'
        else:
            self.stopping = True
            threading.Thread(target=self._stop_and_clear, daemon=True).start()
            response.success, response.message = True, f"Stopping: {', '.join(n for n in STOP_ORDER if n in self.procs)}."
        return response

    def status_cb(self, request, response):
        response.success = True
        if self.running():
            state = 'STOP_REQUESTED' if self.stopping else 'RUNNING'
            alive = ','.join(self._alive())
            response.message = (f'{state}; running={alive}; elapsed={time.monotonic() - self.started_at:.1f}s; '
                                f'logs={self.log_prefix}_*.log; does NOT imply task success')
        else:
            response.message = (f'STOPPED; last_exit_code={self.last_exit}; logs={self.log_prefix}_*.log'
                                + (f'; photos={self.photos}' if self.photos else ''))
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
            main = self.procs.get(self.main)
            self.last_exit = None if main is None else main.returncode
            self.started_at, self.stopping = None, False

    def _wait(self, names, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and any(self.procs[n].poll() is None for n in names if n in self.procs):
            time.sleep(0.2)

    def _snapshot(self):
        """Pictures after a recorder policy, from its still-running stream: exec has stopped, so the robot is still."""
        stream = self.procs.get('stream')
        if not self.snapshot_cameras or stream is None or stream.poll() is not None or self.started_at is None:
            return
        out = self.log_dir / 'snapshots' / Path(self.log_prefix).name
        python = Path(os.path.expanduser(self.repo)) / '.venv' / 'bin' / 'python'
        try:
            proc = subprocess.run([str(python), str(SNAPSHOT), '--out', str(out), '--cameras', *self.snapshot_cameras],
                                  capture_output=True, text=True, timeout=20)
            result = json.loads(proc.stdout.strip().splitlines()[-1])
        except (OSError, subprocess.TimeoutExpired, IndexError, ValueError) as exc:
            result = {'error': str(exc)}
        if 'error' in result:
            self.get_logger().warning(f"no picture after the policy: {result['error']}")
        else:
            self.photos = str(out)

    def _shutdown(self, snapshot=True):
        """In STOP_ORDER: Ctrl-C, then SIGTERM. Jetson processes by pkill there, local ones by process group."""
        for name in STOP_ORDER:
            if name == 'stream' and snapshot:
                self._snapshot()
            proc = self.procs.get(name)
            if proc is None or proc.poll() is not None:
                continue
            for sig in ('INT', 'TERM'):
                if name in REMOTE_PATTERNS:
                    subprocess.run(self._ssh(f'pkill -{sig} -f -- {shlex.quote(REMOTE_PATTERNS[name])}'), timeout=15,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                else:
                    try:
                        os.killpg(proc.pid, getattr(signal, f'SIG{sig}'))
                    except ProcessLookupError:
                        break
                self._wait([name], self.stop_timeout)
                if proc.poll() is not None:
                    break
            if proc.poll() is None and name in REMOTE_PATTERNS:
                proc.terminate()   # the local ssh; the remote may need a manual check
                self.get_logger().error(f'{REMOTE_PATTERNS[name].split()[0]} did not stop on {self.jetson_host}; '
                                        'check it there')
            elif proc.poll() is None:
                self.get_logger().error(f'{name} still running (pid {proc.pid}); check it')


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
            node._shutdown(snapshot=False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
