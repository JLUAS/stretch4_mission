import json

import pytest
import yaml

from stretch4_mission import sync_policies
from stretch4_mission.mission_file import load_policies
from stretch4_mission.sync_policies import append, lerobot_pair, new_entries, pair

POLICIES = """runtime_margin_sec: 10.0

policies:
  grab_sponge:
    root: ~/data/grab_sponge
    ckpt: ~/policies/grab_sponge.ckpt

  # Template for a new policy:
  # open_cabinet:
  #   root: ~/data/open_cabinet
"""


def test_checkpoints_pair_with_the_longest_matching_dataset():
    datasets = ['open_cabinet', 'open_cabinet_v2', 'grab_sponge']
    ckpts = ['open_cabinet_v2_dp.ckpt', 'grab_sponge.ckpt', 'open_cabinet.ckpt', 'dp_brush', 'grab_spongeX.ckpt',
             'bad name.ckpt', 'logs']
    assert pair(datasets, ckpts) == [('grab_sponge', 'grab_sponge', 'grab_sponge.ckpt'),
                                     ('open_cabinet', 'open_cabinet', 'open_cabinet.ckpt'),
                                     ('open_cabinet_v2_dp', 'open_cabinet_v2', 'open_cabinet_v2_dp.ckpt')]


def test_lerobot_policy_takes_its_newest_own_dataset():
    poses = ['floor_cabinet_open', 'floor_cabinet_open_v2', 'no_model']
    models = {'floor_cabinet_open': ('diffusion', 48), 'floor_cabinet_open_v2': ('pi05', 50), 'pi05_mt_bin': ('pi05', 50)}
    datasets = ['floor_cabinet_open_20261022_105005', 'floor_cabinet_open_20261020_090000',
                'floor_cabinet_open_v2_20261025_000000', 'grab_sponge']
    assert lerobot_pair(poses, models, datasets) == [('floor_cabinet_open', 'floor_cabinet_open_20261022_105005'),
                                                     ('floor_cabinet_open_v2', 'floor_cabinet_open_v2_20261025_000000')]


def test_new_policies_are_appended_and_existing_ones_kept(tmp_path, monkeypatch):
    f = tmp_path / 'policies.yaml'
    f.write_text(POLICIES)
    for name in ('grab_sponge', 'open_cabinet_v2', 'no_pose'):
        (tmp_path / 'data' / name).mkdir(parents=True)
        if name != 'no_pose':
            (tmp_path / 'data' / name / 'start_pose.json').write_text('{}')
    (tmp_path / 'lerobot' / 'policies' / 'floor_cabinet_open').mkdir(parents=True)
    (tmp_path / 'lerobot' / 'policies' / 'floor_cabinet_open' / 'start_pose.json').write_text('{}')
    meta = tmp_path / 'hf' / 'SilentSammy2001' / 'floor_cabinet_open_20261022_105005' / 'meta'
    meta.mkdir(parents=True)
    (meta / 'info.json').write_text(json.dumps({'fps': 15}))
    monkeypatch.setattr(sync_policies, 'HF_DIR', str(tmp_path / 'hf'))
    lerobot = {'repo': str(tmp_path / 'lerobot'), 'models': {'floor_cabinet_open': ('diffusion', 48)}}

    entries = new_entries(f, str(tmp_path / 'data'), '~/policies',
                          ['grab_sponge.ckpt', 'open_cabinet_v2_dp.ckpt', 'no_pose.ckpt'], lerobot)
    assert [n for n, _ in entries] == ['open_cabinet_v2_dp', 'floor_cabinet_open']
    assert entries[0][1]['root'] == f'{tmp_path}/data/open_cabinet_v2'
    assert entries[1][1] == {'runner': 'lerobot', 'task': 'Floor Cabinet Open',
                             'root': f'{tmp_path}/lerobot/policies/floor_cabinet_open',
                             'ckpt': '~/policies/floor_cabinet_open/pretrained_model',
                             'dataset': f'{tmp_path}/hf/SilentSammy2001/floor_cabinet_open_20261022_105005',
                             'policy_type': 'diffusion', 'max_runtime_sec': 120,
                             'run_args': '--fps=15 --actions_per_chunk=48 --policy_device=cuda '
                                         '--chunk_size_threshold=0.7 --robot.use_base=true'}
    append(f, entries)
    policies = load_policies(f)
    assert set(policies) == {'grab_sponge', 'open_cabinet_v2_dp', 'floor_cabinet_open'}
    assert policies['floor_cabinet_open']['runner'] == 'lerobot' and policies['open_cabinet_v2_dp']['runner'] == 'recorder'
    assert '# Template for a new policy' in f.read_text()   # comments survive
    assert new_entries(f, str(tmp_path / 'data'), '~/policies', ['open_cabinet_v2_dp.ckpt'], lerobot) == []   # 2nd run


def test_append_refuses_when_policies_is_not_the_last_section(tmp_path):
    f = tmp_path / 'policies.yaml'
    f.write_text('policies:\n  a:\n    root: ~/d\n    ckpt: ~/c\nruntime_margin_sec: 5\n')
    with pytest.raises(ValueError, match='not the last section'):
        append(f, [('b', {'root': '~/data/b', 'ckpt': '~/policies/b.ckpt'})])
    assert set(yaml.safe_load(f.read_text())['policies']) == {'a'}   # unchanged
    assert not (tmp_path / 'policies.tmp').exists()
