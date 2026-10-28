import pytest

from stretch4_mission.mission_file import bridge_params, load_policies, longest_episode_sec, parse_goal, validate_mission

RECORDER = {'runner': 'recorder', 'dataset': '', 'policy_type': ''}   # what load_policies fills in
POLICIES = {'pick_sponge': {'task': 'Pick', 'root': '~/data/sponge', 'ckpt': '~/p/sponge.ckpt', 'run_args': '--fp16',
                            'max_runtime_sec': 60.0, **RECORDER},
            'clean_table': {'task': 'Clean', 'root': '~/data/clean', 'ckpt': '~/p/clean.ckpt', 'run_args': '',
                            'max_runtime_sec': 120.0, **RECORDER}}
MAPS = ['/maps/lab.yaml']


def mission(**kw):
    m = {'name': 'clean_1', 'map': '/maps/lab.yaml',
         'steps': [{'policy': 'pick_sponge', 'goal': {'x': '1.5', 'y': 2, 'yaw': -0.9273}},
                   {'policy': 'clean_table', 'goal': {'x': 0, 'y': 0, 'yaw': 3.14}, 'start_pose': ''}]}  # older files
    m.update(kw)
    return m


def test_validate_normalizes_steps():
    out = validate_mission(mission(), POLICIES, MAPS)
    assert out['steps'][0] == {'type': 'policy', 'policy': 'pick_sponge', 'start_pose': 'start_pose.json',
                               'goal': {'x': 1.5, 'y': 2.0, 'yaw': -0.9273}, 'scan': ''}
    assert out['steps'][1]['start_pose'] == 'start_pose.json'


def test_validate_keeps_selected_start_pose():
    data = mission()
    data['steps'][0]['start_pose'] = 'pickup_left.json'
    assert validate_mission(data, POLICIES, MAPS)['steps'][0]['start_pose'] == 'pickup_left.json'


@pytest.mark.parametrize('bad', [
    {'name': '../etc/x'},                                   # path traversal in the file name
    {'map': '/etc/passwd'},                                 # map must come from the maps list
    {'steps': []},
    {'steps': [{'policy': 'nope', 'goal': {'x': 0, 'y': 0, 'yaw': 0}}]},
    {'steps': [{'policy': 'pick_sponge', 'goal': {'x': 'nan', 'y': 0, 'yaw': 0}}]},
    {'steps': [{'policy': 'pick_sponge', 'goal': {'x': 0, 'y': 0}}]},          # yaw missing
    {'steps': [{'policy': 'pick_sponge', 'goal': {'x': 0, 'y': 0, 'yaw': 0}, 'start_pose': '/p/x.json'}]},
    {'steps': [{'policy': 'pick_sponge', 'goal': {'x': 0, 'y': 0, 'yaw': 0}, 'scan': '../../etc/x.json'}]},  # path in scan name
    {'steps': [{'type': 'dance'}]},
    {'steps': [{'type': 'navigate', 'goal': {'x': 0, 'y': 0}}]},                 # navigate needs a full goal
    {'steps': [{'type': 'command', 'command': '   '}]},
    {'steps': [{'type': 'command', 'command': 'ls\nrm -rf ~'}]},                  # one line only
    {'steps': [{'type': 'command', 'command': 'x' * 1001}]},
    {'steps': [{'type': 'command', 'command': 'ls', 'timeout_sec': 0}]},
])
def test_validate_rejects(bad):
    with pytest.raises(ValueError):
        validate_mission(mission(**bad), POLICIES, MAPS)


def test_validate_navigate_and_command_steps():
    data = mission(steps=[
        {'type': 'navigate', 'goal': {'x': 1, 'y': 2, 'yaw': 0.5}, 'scan': 'scan_1.json', 'policy': 'ignored'},
        {'type': 'command', 'command': '  python3 ~/wave.py --n 2 ', 'goal': {'x': 9}},
        {'type': 'command', 'command': 'ls', 'timeout_sec': '30'},
        {'policy': 'pick_sponge', 'goal': {'x': 0, 'y': 0, 'yaw': 0}}])
    out = validate_mission(data, POLICIES, MAPS)['steps']
    assert out[0] == {'type': 'navigate', 'goal': {'x': 1.0, 'y': 2.0, 'yaw': 0.5}, 'scan': 'scan_1.json'}
    assert out[1] == {'type': 'command', 'command': 'python3 ~/wave.py --n 2'}   # no goal: it runs in place
    assert out[2]['timeout_sec'] == 30.0
    assert out[3]['type'] == 'policy'


def test_lerobot_policies_need_dataset_type_and_seconds(tmp_path):
    f = tmp_path / 'p.yaml'
    good = ('policies:\n  fc:\n    runner: lerobot\n    root: ~/repos/l/policies/fc\n    ckpt: ~/policies/fc/pretrained_model\n'
            '    dataset: ~/.cache/huggingface/lerobot/u/fc_1\n    policy_type: pi05\n    max_runtime_sec: 90\n'
            '    run_args: --fps=30 --robot.use_base=true\n')
    f.write_text(good)
    assert load_policies(f)['fc']['runner'] == 'lerobot' and load_policies(f)['fc']['policy_type'] == 'pi05'
    for bad in (good.replace('runner: lerobot', 'runner: ros'),
                good.replace('    policy_type: pi05\n', ''),
                good.replace('max_runtime_sec: 90', 'max_runtime_sec: auto'),             # auto reads recorder episodes
                good.replace('--fps=30', '--task=other')):                                 # the bridge sets --task
        f.write_text(bad)
        with pytest.raises(ValueError):
            load_policies(f)


def test_bridge_params_sends_every_key():
    # every step sets all bridge parameters, so nothing is left over from the previous policy
    params = bridge_params(POLICIES['clean_table'], '/data/clean/pose_2.json')
    assert set(params) == {'task', 'root', 'pose_file', 'ckpt', 'run_args', 'max_runtime_sec', 'runner', 'dataset',
                           'policy_type'}
    assert params['pose_file'] == '/data/clean/pose_2.json'


def test_load_policies_defaults_types_and_required(tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text('policies:\n  a:\n    root: ~/data/a\n    ckpt: ~/a.ckpt\n    max_runtime_sec: 90\n')
    assert load_policies(f)['a'] == {'task': '', 'root': '~/data/a', 'ckpt': '~/a.ckpt', 'run_args': '',
                                     'max_runtime_sec': 90.0, 'runtime_margin_sec': 10.0, 'recorded': None, **RECORDER}
    assert isinstance(load_policies(f)['a']['max_runtime_sec'], float)  # the bridge declares it as double
    for bad in ('policies:\n  a:\n    root: ~/data/a\n',                                  # ckpt missing
                'policies:\n  a:\n    root: ~/d\n    ckpt: ~/c\n    fps: 30\n'):          # unknown key
        f.write_text(bad)
        with pytest.raises(ValueError):
            load_policies(f)


def episode(root, name, times, tail=''):
    d = root / name
    d.mkdir(parents=True)
    (d / 'records.jsonl').write_text(''.join(f'{{"i": {i}, "t": {t}}}\n' for i, t in enumerate(times)) + tail)


def test_auto_runtime_is_longest_episode_plus_margin(tmp_path):
    episode(tmp_path, 'episode_a', [100.0, 101.0, 113.5])        # 13.5 s
    episode(tmp_path, 'episode_b', [2000.0, 2026.25], '\n')       # 26.25 s, trailing blank line
    episode(tmp_path, 'episode_c', [5.0], '{"i": 1, "t": ')        # partial last line: skipped
    episode(tmp_path, '_pending', [0.0, 999.0])                    # not an episode_* folder
    assert longest_episode_sec(tmp_path) == 26.25
    f = tmp_path / 'p.yaml'
    f.write_text(f'runtime_margin_sec: 5\npolicies:\n  a:\n    root: {tmp_path}\n    ckpt: ~/a.ckpt\n'
                 f'    max_runtime_sec: auto\n')
    params = bridge_params(load_policies(f)['a'])
    assert params['max_runtime_sec'] == 31.25 and isinstance(params['max_runtime_sec'], float)
    assert 'runtime_margin_sec' not in params                      # not a bridge parameter
    with pytest.raises(ValueError):
        longest_episode_sec(tmp_path / 'empty')
    f.write_text('runtime_margin_sec: -1\npolicies:\n  a:\n    root: ~/d\n    ckpt: ~/c\n')
    with pytest.raises(ValueError):
        load_policies(f)


def test_step_runtime_overrides_the_policy():
    m = validate_mission(mission(steps=[{'policy': 'pick_sponge', 'goal': {'x': 0, 'y': 0, 'yaw': 0}, 'max_runtime_sec': '45'},
                                        {'policy': 'clean_table', 'goal': {'x': 0, 'y': 0, 'yaw': 0}, 'max_runtime_sec': ''}]),
                         POLICIES, MAPS)
    assert m['steps'][0]['max_runtime_sec'] == 45.0 and 'max_runtime_sec' not in m['steps'][1]   # empty = policy's
    assert bridge_params(POLICIES['pick_sponge'], '', m['steps'][0]['max_runtime_sec'])['max_runtime_sec'] == 45.0
    assert bridge_params(POLICIES['pick_sponge'])['max_runtime_sec'] == 60.0
    for bad in (0, -5, 'x', 5000):
        with pytest.raises(ValueError, match='max_runtime_sec'):
            validate_mission(mission(steps=[{'policy': 'pick_sponge', 'goal': {'x': 0, 'y': 0, 'yaw': 0},
                                             'max_runtime_sec': bad}]), POLICIES, MAPS)


def test_margin_per_policy_overrides_the_shared_one(tmp_path):
    episode(tmp_path, 'episode_a', [0.0, 20.0])
    f = tmp_path / 'p.yaml'
    common = f'    root: {tmp_path}\n    ckpt: ~/c\n    max_runtime_sec: auto\n'
    f.write_text(f'runtime_margin_sec: 10\npolicies:\n  a:\n{common}    runtime_margin_sec: 2.5\n  b:\n{common}')
    lib = load_policies(f)
    assert bridge_params(lib['a'])['max_runtime_sec'] == 22.5   # own margin
    assert bridge_params(lib['b'])['max_runtime_sec'] == 30.0   # shared margin
    f.write_text(f'policies:\n  a:\n{common}    runtime_margin_sec: -3\n')
    with pytest.raises(ValueError, match='policy a'):
        load_policies(f)


def test_shipped_policy_library_loads():
    from pathlib import Path
    root = Path(__file__).parents[1]
    assert load_policies(root / 'config' / 'policies.yaml')['grab_sponge']['run_args'] == '--fp16 --base'


def test_ui_shows_fixed_and_auto_runtime(tmp_path):
    from stretch4_mission.mission_ui import MissionUI
    base = {'root': str(tmp_path), 'runtime_margin_sec': 10.0}
    assert MissionUI.policy_runtime({**base, 'max_runtime_sec': 45.0}) == {'sec': 45.0, 'note': 'fixed max_runtime_sec'}
    assert MissionUI.policy_runtime({**base, 'max_runtime_sec': 'auto'})['sec'] is None   # no episodes: shown as unknown
    episode(tmp_path, 'episode_a', [0.0, 26.0])
    assert MissionUI.policy_runtime({**base, 'max_runtime_sec': 'auto'}) == {
        'sec': 36.0, 'note': 'longest recording 26.0 s (1 episodes) + 10 s'}


def test_parse_goal_for_go_to_pose():
    assert parse_goal('1.5,-2,0.7854') == {'x': 1.5, 'y': -2.0, 'yaw': 0.7854}
    for bad in ('1,2', '1,2,3,4', '1,x,0', ',,', '1,2,nan', '5000,0,0'):
        with pytest.raises(ValueError):
            parse_goal(bad)


def test_step_run_args_override_the_policy():
    step = {'policy': 'pick_sponge', 'goal': {'x': 0, 'y': 0, 'yaw': 0}}
    args = '--num-inference-steps 5 --fp16 --schedule async --base'
    m = validate_mission(mission(steps=[{**step, 'run_args': f'  {args} '}, {**step, 'run_args': ''}]), POLICIES, MAPS)
    assert m['steps'][0]['run_args'] == args and 'run_args' not in m['steps'][1]   # empty = the policy's
    assert bridge_params(POLICIES['pick_sponge'], '', None, args)['run_args'] == args
    assert bridge_params(POLICIES['pick_sponge'])['run_args'] == '--fp16'
    for bad in ('--yes', '--duration 9', '--ckpt=/x.ckpt', '--fp16 "unclosed', 'x' * 301):
        with pytest.raises(ValueError, match='run_args'):
            validate_mission(mission(steps=[{**step, 'run_args': bad}]), POLICIES, MAPS)


def test_saved_runtimes_replace_deleted_or_cut_off_episodes(tmp_path):
    import json
    import shutil
    from stretch4_mission.mission_file import RUNTIMES_FILE, recorded_runtime, save_recorded_runtimes
    data = tmp_path / 'data'
    episode(data, 'episode_a', [0.0, 20.0])
    episode(data, 'episode_b', [0.0, 68.7])
    f = tmp_path / 'policies.yaml'
    f.write_text(f'policies:\n  clean:\n    root: {data}\n    ckpt: ~/c\n    max_runtime_sec: auto\n')
    assert save_recorded_runtimes(f) == ['clean: 68.7 s (2 episodes)']
    saved = json.loads((tmp_path / RUNTIMES_FILE).read_text())['clean']
    assert saved['longest_episode_sec'] == 68.7 and saved['episodes'] == 2
    assert save_recorded_runtimes(f) == []                                     # nothing new: file untouched
    shutil.rmtree(data / 'episode_b')
    episode(data, 'episode_b', [0.0], '{"i": 1, "t": ')                        # copied incompletely
    assert save_recorded_runtimes(f) == []                                     # a cut-off copy never overwrites
    assert recorded_runtime(load_policies(f)['clean'])[0] == 68.7              # ... and the saved length is used
    shutil.rmtree(data / 'episode_a'), shutil.rmtree(data / 'episode_b')     # episodes deleted
    sec, source = recorded_runtime(load_policies(f)['clean'])
    assert sec == 68.7 and RUNTIMES_FILE in source
    assert bridge_params(load_policies(f)['clean'])['max_runtime_sec'] == 78.7
    (tmp_path / RUNTIMES_FILE).write_text('{"clean": {"longest_episode_sec": -1}, "x": 5}')   # bad entries ignored
    with pytest.raises(ValueError, match='none saved'):
        recorded_runtime(load_policies(f)['clean'])


def test_a_partial_copy_does_not_replace_the_saved_length(tmp_path):
    from stretch4_mission.mission_file import recorded_runtime, save_recorded_runtimes
    data = tmp_path / 'data'
    for i, end in enumerate((20.0, 68.7, 30.0)):
        episode(data, f'episode_{i}', [0.0, end])
    f = tmp_path / 'policies.yaml'
    f.write_text(f'policies:\n  clean:\n    root: {data}\n    ckpt: ~/c\n    max_runtime_sec: auto\n')
    save_recorded_runtimes(f)                                    # where it was recorded: 3 episodes, 68.7 s
    import shutil
    shutil.rmtree(data / 'episode_1'), shutil.rmtree(data / 'episode_2')   # another robot got only 1 (complete) episode
    assert save_recorded_runtimes(f) == []                       # fewer episodes than saved: not overwritten
    sec, source = recorded_runtime(load_policies(f)['clean'])
    assert sec == 68.7 and 'saved' in source
    for i, end in enumerate((40.0, 90.0, 10.0, 5.0), start=10):  # more episodes than saved: a newer dataset
        episode(data, f'episode_{i}', [0.0, end])
    assert save_recorded_runtimes(f) == ['clean: 90.0 s (5 episodes)']
    assert recorded_runtime(load_policies(f)['clean'])[0] == 90.0
