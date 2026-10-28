import shlex
import subprocess

from stretch4_mission.policy_bridge import (STOP_ORDER, lerobot_client_command, policy_pose_file, remote_path,
                                          run_command, venv_command)


def test_remote_command_keeps_tilde_for_the_jetson_and_quotes_the_rest():
    cmd = venv_command('~/repos/stretch4_policy_recorder', run_command('~/policies/grab sponge.ckpt', '--fp16 --base'),
                       remote=True)
    assert cmd == ('cd "$HOME"/repos/stretch4_policy_recorder && . .venv/bin/activate && '
                   'exec stretch4_policy_run --ckpt "$HOME"/\'policies/grab sponge.ckpt\' --fp16 --base --yes')
    assert run_command('/p/x.ckpt', "--name 'a b'").endswith("--ckpt /p/x.ckpt --name 'a b' --yes")
    # --yes: no terminal to answer "Go?"; --duration: the policy stops on its own at max_runtime_sec
    assert run_command('/p/x.ckpt', '--fp16', 120.0).endswith('--fp16 --yes --duration 120')
    assert remote_path('~') == '"$HOME"' and remote_path('/a;rm -rf x') == "'/a;rm -rf x'"


def test_remote_path_expands_in_a_real_shell(tmp_path):
    # what the Jetson's shell will see: ~ expands, spaces and ; stay inside the argument
    out = subprocess.run(['sh', '-c', 'printf "%s" ' + remote_path('~/a b;c')], capture_output=True, text=True,
                         env={'HOME': str(tmp_path)}).stdout
    assert out == f'{tmp_path}/a b;c'


def test_local_venv_command_expands_on_the_robot():
    cmd = venv_command('~/repos/r', 'stretch4_policy_exec')
    assert '~' not in cmd and cmd.endswith('&& . .venv/bin/activate && exec stretch4_policy_exec')
    assert shlex.split(cmd)[0] == 'cd'


def test_policy_pose_file_stays_in_policy_root(tmp_path):
    pose = tmp_path / 'pose_2.json'
    assert policy_pose_file(tmp_path, 'pose_2.json') == pose
    assert policy_pose_file(tmp_path, pose) == pose
    try:
        policy_pose_file(tmp_path, '../outside.json')
    except ValueError:
        pass
    else:
        raise AssertionError('pose path traversal accepted')


def test_lerobot_client_is_the_documented_command():
    cmd = lerobot_client_command(
        '/home/hello-robot/.cache/huggingface/lerobot/SilentSammy2001/floor_cabinet_open_20261022_105005',
        'Floor Cabinet Open', '192.168.1.101:8080', '/home/jetson1/policies/floor_cabinet_open/pretrained_model',
        'diffusion', '--fps=30 --actions_per_chunk=48 --policy_device=cuda --chunk_size_threshold=0.7 --robot.use_base=true')
    assert shlex.split(cmd) == [
        'python', '-m', 'lerobot.async_inference.robot_client', '--robot.type=stretch4', '--robot.mode=control',
        '--robot.preprocessing_from=/home/hello-robot/.cache/huggingface/lerobot/SilentSammy2001/floor_cabinet_open_20261022_105005',
        '--task=Floor Cabinet Open', '--server_address=192.168.1.101:8080',
        '--pretrained_name_or_path=/home/jetson1/policies/floor_cabinet_open/pretrained_model', '--policy_type=diffusion',
        '--fps=30', '--actions_per_chunk=48', '--policy_device=cuda', '--chunk_size_threshold=0.7', '--robot.use_base=true']


def test_what_sends_actions_stops_first():
    assert STOP_ORDER.index('run') < STOP_ORDER.index('exec') < STOP_ORDER.index('stream')
    assert STOP_ORDER.index('client') < STOP_ORDER.index('server')
