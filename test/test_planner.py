import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from stretch4_mission import planner
from stretch4_mission.mission_file import (CONFIGS_FILE, load_configs, load_mission, stored, validate_config,
                                           validate_mission, write_config)
from stretch4_mission.planner import build_catalog, plan_schema, plan_to_mission

POLICIES = {'grab_sponge': {'task': 'Grab the sponge', 'root': '~/d/g', 'ckpt': '~/p/g.ckpt', 'run_args': '',
                            'max_runtime_sec': 60.0, 'runner': 'recorder', 'dataset': '', 'policy_type': ''},
            'clean_table': {'task': 'Clean the table', 'root': '~/d/c', 'ckpt': '~/p/c.ckpt', 'run_args': '',
                            'max_runtime_sec': 90.0, 'runner': 'recorder', 'dataset': '', 'policy_type': ''}}
MAP, OTHER = '/maps/lab.yaml', '/maps/home.yaml'
CONFIGS = {'cabinet': validate_config('cabinet', {'map': MAP, 'policy': 'grab_sponge', 'label': 'kitchen cabinet',
                                                  'goal': {'x': 1, 'y': 2, 'yaw': 0}, 'scan': 'scan_1.json'}, POLICIES),
           'table': validate_config('table', {'map': MAP, 'policy': 'clean_table', 'max_runtime_sec': 45,
                                              'goal': {'x': 3, 'y': 0, 'yaw': 1}}, POLICIES)}
KITCHEN = {'type': 'navigate', 'goal': {'x': 5, 'y': 1, 'yaw': 3.1}, 'label': 'kitchen'}


def missions():
    raw = [{'name': 'clean', 'map': MAP, 'steps': [{'config': 'cabinet'}, {'type': 'policy', 'config': 'table'}]},
           {'name': 'tour', 'map': MAP, 'steps': [KITCHEN, {'type': 'command', 'command': 'python3 ~/wave.py', 'label': 'wave'},
                                                  {'config': 'cabinet'}]}]
    return [validate_mission(m, POLICIES, [MAP], CONFIGS) for m in raw]


def test_a_config_step_is_resolved_and_saved_as_its_name():
    step = missions()[0]['steps'][1]
    assert step == {'type': 'policy', 'config': 'table', 'policy': 'clean_table', 'start_pose': 'start_pose.json',
                    'max_runtime_sec': 45.0, 'goal': {'x': 3.0, 'y': 0.0, 'yaw': 1.0}, 'scan': ''}
    assert stored(missions()[0])['steps'] == [{'type': 'policy', 'config': 'cabinet'}, {'type': 'policy', 'config': 'table'}]
    with pytest.raises(ValueError, match='unknown policy config'):
        validate_mission({'name': 'x', 'map': MAP, 'steps': [{'config': 'gone'}]}, POLICIES, [MAP], CONFIGS)
    with pytest.raises(ValueError, match='is on lab.yaml, the mission on home.yaml'):
        validate_mission({'name': 'x', 'map': OTHER, 'steps': [{'config': 'cabinet'}]}, POLICIES, [MAP, OTHER], CONFIGS)


def test_configs_are_written_read_and_deleted(tmp_path):
    path = tmp_path / CONFIGS_FILE
    write_config(path, 'cabinet', CONFIGS['cabinet'])
    path.write_text(path.read_text() + '  broken:\n    map: /maps/lab.yaml\n')   # a bad entry is reported, not fatal
    good, bad = load_configs(path, POLICIES)
    assert good == {'cabinet': CONFIGS['cabinet']} and set(bad) == {'broken'}
    write_config(path, 'cabinet', None)
    assert load_configs(path, POLICIES)[0] == {} and load_configs(tmp_path / 'none.yaml', POLICIES) == ({}, {})
    for bad_cfg in ({'map': MAP, 'policy': 'nope', 'goal': {'x': 0, 'y': 0, 'yaw': 0}},
                    {'map': MAP, 'policy': 'grab_sponge'},                                   # no goal
                    {'map': 'x.png', 'policy': 'grab_sponge', 'goal': {'x': 0, 'y': 0, 'yaw': 0}}):
        with pytest.raises(ValueError):
            validate_config('c', bad_cfg, POLICIES)


def test_a_mission_file_finds_its_configs(tmp_path):
    write_config(tmp_path / CONFIGS_FILE, 'table', CONFIGS['table'])
    (tmp_path / 'm.yaml').write_text(json.dumps({'name': 'm', 'map': MAP, 'steps': [{'config': 'table'}]}))
    assert load_mission(tmp_path / 'm.yaml', POLICIES)['steps'][0]['policy'] == 'clean_table'


def test_catalog_is_the_configs_plus_saved_places_commands_and_missions():
    catalog = build_catalog(missions(), POLICIES, CONFIGS)
    assert [c['id'] for c in catalog] == ['config:cabinet', 'config:table', 'mission:clean',
                                          'navigate:tour:1', 'command:tour:2', 'mission:tour']
    by_id = {c['id']: c['description'] for c in catalog}
    assert by_id['config:cabinet'] == 'Run policy grab_sponge (Grab the sponge) at kitchen cabinet'
    assert by_id['config:table'] == 'Run policy clean_table (Clean the table) at table'
    assert by_id['navigate:tour:1'] == 'Go to kitchen'
    assert by_id['command:tour:2'] == 'Run the command "python3 ~/wave.py" (wave)'   # the saved command, word for word
    assert by_id['mission:tour'].endswith('; then Run policy grab_sponge (Grab the sponge) at kitchen cabinet')
    assert [c['id'] for c in build_catalog([], POLICIES, CONFIGS)] == ['config:cabinet', 'config:table']   # no missions


def test_config_notes_reach_the_catalog_but_not_the_mission():
    cfg = validate_config('table2', {'map': MAP, 'policy': 'clean_table', 'goal': {'x': 0, 'y': 0, 'yaw': 0},
                                     'needs': ' a sponge held in the gripper ', 'leaves': 'the table clean'}, POLICIES)
    assert cfg['needs'] == 'a sponge held in the gripper'
    desc = build_catalog([], POLICIES, {'table2': cfg})[0]['description']
    assert desc == ('Run policy clean_table (Clean the table) at table2. Needs: a sponge held in the gripper. '
                    'Leaves: the table clean')
    step = validate_mission({'name': 'n', 'map': MAP, 'steps': [{'config': 'table2'}]}, POLICIES, [MAP], {'table2': cfg})
    assert 'needs' not in step['steps'][0] and 'leaves' not in step['steps'][0]
    with pytest.raises(ValueError, match='needs'):
        validate_config('t', {'map': MAP, 'policy': 'clean_table', 'goal': {'x': 0, 'y': 0, 'yaw': 0}, 'needs': 'a\nb'},
                        POLICIES)


def test_schema_only_allows_catalog_ids():
    schema = plan_schema(['a', 'b'])
    assert schema['properties']['plan']['items']['enum'] == ['a', 'b']
    assert schema['additionalProperties'] is False and set(schema['required']) == {'plan', 'message'}


def test_plan_becomes_a_valid_mission_that_refers_to_the_configs():
    catalog = build_catalog(missions(), POLICIES, CONFIGS)
    m = plan_to_mission(['navigate:tour:1', 'config:table', 'mission:clean', 'command:tour:2'], catalog, 'ask_1', MAP)
    out = validate_mission(m, POLICIES, [MAP], CONFIGS)
    assert [s.get('config') or s['type'] for s in out['steps']] == ['navigate', 'table', 'cabinet', 'table', 'command']
    assert stored(out)['steps'][1] == {'type': 'policy', 'config': 'table'}
    with pytest.raises(ValueError, match='not in the catalog'):
        plan_to_mission(['command:tour:9'], catalog, 'ask_2', MAP)


def test_step_labels_are_optional_and_one_line():
    m = validate_mission({'name': 'l', 'map': MAP, 'steps': [{**KITCHEN, 'label': '  kitchen  '}, {**KITCHEN, 'label': ''}]},
                         POLICIES, [MAP])
    assert m['steps'][0]['label'] == 'kitchen' and 'label' not in m['steps'][1]
    for bad in ('a\nb', 'x' * 61):
        with pytest.raises(ValueError):
            validate_mission({'name': 'l', 'map': MAP, 'steps': [{**KITCHEN, 'label': bad}]}, POLICIES, [MAP])


def test_planner_without_a_key_explains_how_to_set_one(tmp_path):
    env = {k: v for k, v in os.environ.items() if not k.startswith('ANTHROPIC_')}
    env['HOME'] = str(tmp_path)
    proc = subprocess.run([sys.executable, str(Path(planner.__file__))], input=json.dumps({'text': 'x', 'catalog': [],
                          'model': 'm'}), capture_output=True, text=True, env=env)
    assert proc.returncode == 1 and 'no Anthropic API key found' in json.loads(proc.stdout.splitlines()[-1])['error']
