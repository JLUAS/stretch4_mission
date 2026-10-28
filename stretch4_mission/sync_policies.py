#!/usr/bin/env python3
"""Add every policy that can run here (start pose on the robot, model on the Jetson) to config/policies.yaml.

    ros2 run stretch4_mission sync_policies     # scripts/run_ui_robot.sh runs it before the UI

runner recorder: a checkpoint <name>.ckpt or <name>_<anything>.ckpt in the Jetson's ~/policies pairs with the dataset
~/data/<name> (the longest matching name wins) that has a start_pose.json. The policy is named after the checkpoint.

runner lerobot: ~/repos/lerobot_stretch4/policies/<name>/start_pose.json on the robot pairs with the Jetson model
~/policies/<name>/pretrained_model (its config.json gives policy_type and actions per chunk) and the newest robot dataset
~/.cache/huggingface/lerobot/<user>/<name>[_<anything>] (its meta/info.json gives fps). The dataset is matched by name:
check it in policies.yaml.

Entries already in policies.yaml (same name or same model) are never changed; edit them there.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml

from stretch4_mission.mission_file import NAME_RE, load_policies

POLICIES_FILE = '~/ament_ws/src/stretch4_mission/config/policies.yaml'   # same default as mission_ui
# shortcut: the default $HF_LEROBOT_HOME only; add an option if a robot keeps its datasets elsewhere
HF_DIR = '~/.cache/huggingface/lerobot'
LEROBOT_RUN_ARGS = '--fps={fps} --actions_per_chunk={n} --policy_device=cuda --chunk_size_threshold=0.7 --robot.use_base=true'


def owner(dataset, names):
    """The longest name that dataset is, or starts with + '_' (floor_cabinet_open_v2_1 -> floor_cabinet_open_v2)."""
    hits = [n for n in names if dataset == n or dataset.startswith(n + '_')]
    return max(hits, key=len) if hits else None


def pair(datasets, checkpoints):
    """Recorder: dataset names, checkpoint file names -> [(policy name, dataset, checkpoint)]."""
    out = []
    for ckpt in sorted(checkpoints):
        stem = ckpt[:-len('.ckpt')] if ckpt.endswith('.ckpt') else None
        if stem and NAME_RE.match(stem) and owner(stem, datasets):
            out.append((stem, owner(stem, datasets), ckpt))
    return out


def lerobot_pair(poses, models, datasets):
    """LeRobot: start-pose folder names, {name: (type, n_action_steps)}, dataset folder names -> [(name, dataset)]."""
    out = []
    for name in sorted(set(poses) & set(models)):
        mine = sorted(d for d in datasets if owner(d, poses) == name)   # newest last: names end in a timestamp
        if mine and NAME_RE.match(name):
            out.append((name, mine[-1]))
    return out


def new_entries(policies_file, data_dir, jetson_dir, checkpoints, lerobot=None):
    """Policies that policies.yaml does not have yet -> [(name, {key: value})], ~ paths as the file writes them.

    lerobot: {'repo': robot lerobot_stretch4 repo, 'models': {name: (type, n_action_steps)}} or None.
    """
    known = load_policies(policies_file)
    known_ckpts = {p['ckpt'] for p in known.values()} | {Path(p['ckpt']).name for p in known.values()}
    data = Path(data_dir).expanduser()
    datasets = [d.name for d in data.iterdir() if (d / 'start_pose.json').is_file()] if data.is_dir() else []
    out = [(name, {'task': name.replace('_', ' ').capitalize(), 'root': f'{data_dir}/{dataset}',
                   'ckpt': f'{jetson_dir}/{ckpt}', 'run_args': '--fp16 --base', 'max_runtime_sec': 'auto'})
           for name, dataset, ckpt in pair(datasets, checkpoints)]
    if lerobot:
        repo = f"{lerobot['repo']}/policies"
        poses = [d.name for d in Path(repo).expanduser().glob('*') if (d / 'start_pose.json').is_file()]
        hf = {d.parent.name: d.parent for d in Path(HF_DIR).expanduser().glob('*/*/meta')}   # dataset name -> folder
        for name, dataset in lerobot_pair(poses, lerobot['models'], list(hf)):
            policy_type, n = lerobot['models'][name]
            fps = json.loads((hf[dataset] / 'meta' / 'info.json').read_text()).get('fps', 30)
            out.append((name, {'runner': 'lerobot', 'task': name.replace('_', ' ').title(), 'root': f'{repo}/{name}',
                               'ckpt': f'{jetson_dir}/{name}/pretrained_model',
                               'dataset': f'{HF_DIR}/{hf[dataset].parent.name}/{dataset}', 'policy_type': policy_type,
                               'run_args': LEROBOT_RUN_ARGS.format(fps=fps, n=n or 48), 'max_runtime_sec': 120}))
    taken = set(known)
    keep = []
    for name, fields in out:
        if name not in taken and fields['ckpt'] not in known_ckpts and Path(fields['ckpt']).name not in known_ckpts:
            taken.add(name)
            keep.append((name, fields))
    return keep


def append(policies_file, entries):
    """Add entries at the end of the policies: mapping, keeping the file's comments; checked before it is replaced."""
    path = Path(policies_file)
    text = path.read_text().rstrip('\n') + '\n'
    for name, fields in entries:
        text += f"\n  {name}:   # added by sync_policies {time.strftime('%Y-%m-%d')}\n"
        text += ''.join(f'    {k}: {v}\n' for k, v in fields.items())
    tmp = path.with_suffix('.tmp')
    tmp.write_text(text)
    try:
        ok = {n for n, _ in entries} <= set(load_policies(tmp))   # the whole file still loads, new entries included
    except (ValueError, yaml.YAMLError):
        ok = False
    if not ok:
        tmp.unlink()
        raise ValueError(f'{path}: "policies:" is not the last section, add these by hand: {entries}')
    tmp.replace(path)   # atomic: the UI and a mission may read it at any time


def jetson_listing(runner_file, jetson_dir):
    """-> (runner config, file names in jetson_dir, {name: (policy type, n_action_steps)} of its LeRobot models)."""
    with open(runner_file) as f:
        cfg = yaml.safe_load(f)['stretch4_policy_bridge']['ros__parameters']
    read = ('import json,sys; c=json.load(open(sys.argv[1])); '
            'print(sys.argv[2], c.get("type", ""), c.get("n_action_steps") or "")')
    # jetson_dir unquoted: the Jetson's shell expands its ~
    script = (f'ls -1 -- {jetson_dir}; echo ---; for d in {jetson_dir}/*/pretrained_model; do '
              f'[ -f "$d/config.json" ] && python3 -c {json.dumps(read)} "$d/config.json" "$(basename "$(dirname "$d")")"; '
              'done; true')
    proc = subprocess.run(['ssh', *cfg['ssh_options'], cfg['jetson_host'], script], capture_output=True, text=True,
                          timeout=30)
    if proc.returncode != 0:
        raise RuntimeError(f"cannot list {jetson_dir} on {cfg['jetson_host']}: {proc.stderr.strip() or proc.returncode}")
    files, _, models = proc.stdout.partition('---\n')
    parsed = {}
    for line in models.splitlines():
        parts = line.split()
        if len(parts) >= 2:
            parsed[parts[0]] = (parts[1], int(parts[2]) if len(parts) > 2 and parts[2].isdigit() else None)
    return cfg, files.split(), parsed


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--policies-file', default=POLICIES_FILE, help='policy_runner.yaml is read next to it')
    ap.add_argument('--data-dir', default='~/data', help='recorder datasets on the robot')
    ap.add_argument('--jetson-dir', default='~/policies', help='checkpoints and models on the Jetson')
    a = ap.parse_args()
    a.policies_file = Path(a.policies_file).expanduser()
    try:
        cfg, ckpts, models = jetson_listing(a.policies_file.with_name('policy_runner.yaml'), a.jetson_dir)
        lerobot = {'repo': cfg.get('lerobot_robot_repo', '~/repos/lerobot_stretch4'), 'models': models}
        entries = new_entries(a.policies_file, a.data_dir, a.jetson_dir, ckpts, lerobot)
        if entries:
            append(a.policies_file, entries)
    except (OSError, ValueError, RuntimeError, KeyError, subprocess.TimeoutExpired, yaml.YAMLError) as exc:
        print(f'sync_policies: {exc}; policies.yaml unchanged', file=sys.stderr)
        return 1
    for name, fields in entries:
        how = f"LeRobot {fields['policy_type']}, dataset {fields['dataset']}" if 'runner' in fields else 'recorder'
        print(f"sync_policies: added {name} ({how}; start pose {fields['root']}, model {fields['ckpt']})")
    policies = load_policies(a.policies_file)
    roots = {Path(os.path.expanduser(p['root'])).resolve() for p in policies.values()}
    folders = [(d, 'recorder dataset') for d in sorted(Path(a.data_dir).expanduser().glob('*'))]
    folders += [(d, 'LeRobot policy') for d in sorted(Path(lerobot['repo'], 'policies').expanduser().glob('*'))]
    for d, kind in folders:
        if d.is_dir() and d.resolve() not in roots:
            why = ('no start_pose.json' if not (d / 'start_pose.json').is_file()
                   else f'no {d.name}.ckpt or {d.name}_*.ckpt in {a.jetson_dir}' if kind == 'recorder dataset'
                   else f'no {a.jetson_dir}/{d.name}/pretrained_model on the Jetson'
                   if d.name not in models else f'no dataset {HF_DIR}/*/{d.name}* on the robot')
            print(f'sync_policies: {kind} {d.name} not added: {why}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
