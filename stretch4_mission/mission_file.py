"""Load and validate mission and policy-library YAML files."""

import json
import math
import os
import re
from pathlib import Path

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
SCAN_RE = re.compile(r'^[A-Za-z0-9_.-]{1,80}\.json$')   # file in <missions_dir>/scans/
POSE_RE = re.compile(r'^[A-Za-z0-9_.-]{1,80}\.json$')   # file directly in the policy root
MAX_STEPS = 20
AUTO = 'auto'   # max_runtime_sec: auto -> longest recorded episode of the policy + runtime_margin_sec
MARGIN_KEY = 'runtime_margin_sec'   # top level (all policies) or per policy; not a bridge parameter
# policy_bridge.py parameter -> (type it is declared with, default). SetParameters needs the exact type, and every
# key is sent at every step so nothing is left over from the previous policy.
POLICY_KEYS = {
    'task': (str, ''),                  # label for logs and the UI
    'root': (str, None),                # robot: dataset folder for stretch4_goto_pose --root (the start pose)
    'ckpt': (str, None),                # Jetson: checkpoint for stretch4_policy_run --ckpt
    'run_args': (str, ''),              # Jetson: extra stretch4_policy_run arguments, e.g. --fp16 --base
    'max_runtime_sec': (float, 120.0),  # the bridge stops the policy after this (s), or "auto"
}


def load_policies(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    policies = data.get('policies')
    if not isinstance(policies, dict) or not policies:
        raise ValueError(f'{path}: expected a non-empty "policies:" mapping')
    margin = _margin(data.get(MARGIN_KEY, 10.0), path)
    out = {}
    for name, cfg in policies.items():
        if not NAME_RE.match(str(name)) or not isinstance(cfg, dict):
            raise ValueError(f'{path}: bad policy entry {name!r}')
        unknown = set(cfg) - set(POLICY_KEYS) - {MARGIN_KEY}
        if unknown:
            raise ValueError(f'{path}: policy {name}: unknown keys {sorted(unknown)} '
                             f'(allowed: {sorted([*POLICY_KEYS, MARGIN_KEY])})')
        missing = [k for k, (_, default) in POLICY_KEYS.items() if default is None and not cfg.get(k)]
        if missing:
            raise ValueError(f'{path}: policy {name}: {", ".join(missing)} required')
        policy = {}
        try:
            for k, (kind, default) in POLICY_KEYS.items():
                value = cfg.get(k, default)
                policy[k] = AUTO if k == 'max_runtime_sec' and value == AUTO else kind(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{path}: policy {name}: {exc}') from None
        policy[MARGIN_KEY] = _margin(cfg[MARGIN_KEY], f'{path}: policy {name}') if MARGIN_KEY in cfg else margin
        out[str(name)] = policy
    return out


def _margin(value, where):
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = math.nan
    if not math.isfinite(value) or value < 0:
        raise ValueError(f'{where}: {MARGIN_KEY} must be a number >= 0')
    return value


def longest_episode_sec(root):
    """Longest episode recorded in a stretch4_policy_recorder dataset: last minus first sample time "t"."""
    longest, count = 0.0, 0
    for records in Path(os.path.expanduser(root)).glob('episode_*/records.jsonl'):
        try:
            with open(records, 'rb') as f:
                first = json.loads(f.readline())['t']
                f.seek(max(0, f.seek(0, os.SEEK_END) - 65536))   # only the tail: the files are ~1 MB
                last = json.loads([line for line in f.read().splitlines() if line.strip()][-1])['t']
        except (OSError, ValueError, KeyError, IndexError, TypeError):
            continue   # unreadable or partial episode: it does not count
        longest, count = max(longest, float(last) - float(first)), count + 1
    if not count:
        raise ValueError(f'max_runtime_sec auto: no recorded episodes (episode_*/records.jsonl) in {root}')
    return longest


def _number(value, what):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{what}: not a number') from None
    if not math.isfinite(value) or abs(value) > 1000.0:
        raise ValueError(f'{what}: out of range')
    return value


def validate_mission(data, policies, maps=None):
    """Return a normalized mission dict or raise ValueError. maps: allowed map paths (UI only)."""
    if not isinstance(data, dict):
        raise ValueError('mission must be a mapping')
    name = str(data.get('name', ''))
    if not NAME_RE.match(name):
        raise ValueError('name: 1-64 letters, digits, "_" or "-"')
    map_path = str(data.get('map', ''))
    if not map_path.endswith('.yaml') or (maps is not None and map_path not in maps):
        raise ValueError(f'map: choose one of the maps in the list ({map_path!r} is not)')
    steps = data.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= MAX_STEPS:
        raise ValueError(f'steps: 1-{MAX_STEPS} steps required')
    out = []
    for i, step in enumerate(steps, 1):
        if not isinstance(step, dict):
            raise ValueError(f'step {i}: must be a mapping')
        policy = str(step.get('policy', ''))
        if policy not in policies:
            raise ValueError(f'step {i}: unknown policy {policy!r}')
        goal = step.get('goal') or {}
        if not isinstance(goal, dict):
            raise ValueError(f'step {i}: goal must be {{x, y, yaw}}')
        start_pose = str(step.get('start_pose') or 'start_pose.json')
        if not POSE_RE.match(start_pose):
            raise ValueError(f'step {i}: bad start pose name {start_pose!r}')
        scan = str(step.get('scan') or '')
        if scan and not SCAN_RE.match(scan):
            raise ValueError(f'step {i}: bad reference scan name {scan!r}')
        entry = {'policy': policy,
                 'start_pose': start_pose,
                 'goal': {k: _number(goal.get(k), f'step {i} goal.{k}') for k in ('x', 'y', 'yaw')},
                 'scan': scan}
        runtime = step.get('max_runtime_sec')
        if runtime not in (None, ''):   # this step's own limit; without it the policy's (policies.yaml) applies
            entry['max_runtime_sec'] = _number(runtime, f'step {i} max_runtime_sec')
            if entry['max_runtime_sec'] <= 0:
                raise ValueError(f'step {i} max_runtime_sec: must be > 0 (leave it empty for the policy default)')
        out.append(entry)
    return {'name': name, 'map': map_path, 'steps': out}


def load_mission(path, policies):
    with open(path) as f:
        return validate_mission(yaml.safe_load(f), policies)


def bridge_params(policy_cfg, pose_file='', max_runtime_sec=None):
    """All policy_bridge parameters for one step (load_policies already filled in the defaults).

    max_runtime_sec: the step's own limit, if the mission sets one. Otherwise the policy's; "auto" becomes the
    policy's longest recorded episode + runtime_margin_sec.
    """
    params = {**{k: policy_cfg[k] for k in POLICY_KEYS}, 'pose_file': str(pose_file)}
    if max_runtime_sec is not None:
        params['max_runtime_sec'] = float(max_runtime_sec)
    elif params['max_runtime_sec'] == AUTO:
        params['max_runtime_sec'] = longest_episode_sec(policy_cfg['root']) + policy_cfg['runtime_margin_sec']
    return params
