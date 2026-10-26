import pytest

from stretch4_mission.mission_file import bridge_params, load_policies, validate_mission

POLICIES = {'pick_sponge': {'task': 'Pick', 'root': '~/data/sponge', 'ckpt': '~/p/sponge.ckpt', 'run_args': '--fp16',
                            'max_runtime_sec': 60.0},
            'clean_table': {'task': 'Clean', 'root': '~/data/clean', 'ckpt': '~/p/clean.ckpt', 'run_args': '',
                            'max_runtime_sec': 120.0}}
MAPS = ['/maps/lab.yaml']


def mission(**kw):
    m = {'name': 'clean_1', 'map': '/maps/lab.yaml',
         'steps': [{'policy': 'pick_sponge', 'goal': {'x': '1.5', 'y': 2, 'yaw': -0.9273}},
                   {'policy': 'clean_table', 'goal': {'x': 0, 'y': 0, 'yaw': 3.14}, 'start_pose': ''}]}  # older files
    m.update(kw)
    return m


def test_validate_normalizes_steps():
    out = validate_mission(mission(), POLICIES, MAPS)
    assert out['steps'][0] == {'policy': 'pick_sponge', 'start_pose': 'start_pose.json',
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
])
def test_validate_rejects(bad):
    with pytest.raises(ValueError):
        validate_mission(mission(**bad), POLICIES, MAPS)


def test_bridge_params_sends_every_key():
    # every step sets all bridge parameters, so nothing is left over from the previous policy
    params = bridge_params(POLICIES['clean_table'], '/data/clean/pose_2.json')
    assert set(params) == {'task', 'root', 'pose_file', 'ckpt', 'run_args', 'max_runtime_sec'}
    assert params['pose_file'] == '/data/clean/pose_2.json'


def test_load_policies_defaults_types_and_required(tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text('policies:\n  a:\n    root: ~/data/a\n    ckpt: ~/a.ckpt\n    max_runtime_sec: 90\n')
    assert load_policies(f)['a'] == {'task': '', 'root': '~/data/a', 'ckpt': '~/a.ckpt', 'run_args': '',
                                     'max_runtime_sec': 90.0}
    assert isinstance(load_policies(f)['a']['max_runtime_sec'], float)  # the bridge declares it as double
    for bad in ('policies:\n  a:\n    root: ~/data/a\n',                                  # ckpt missing
                'policies:\n  a:\n    root: ~/d\n    ckpt: ~/c\n    fps: 30\n'):          # unknown key
        f.write_text(bad)
        with pytest.raises(ValueError):
            load_policies(f)


def test_shipped_policy_library_loads():
    from pathlib import Path
    root = Path(__file__).parents[1]
    assert load_policies(root / 'config' / 'policies.yaml')['grab_sponge']['run_args'] == '--fp16 --base'
