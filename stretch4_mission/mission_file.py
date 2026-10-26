"""Load and validate mission and policy-library YAML files."""

import math
import re

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
SCAN_RE = re.compile(r'^[A-Za-z0-9_.-]{1,80}\.json$')   # file in <missions_dir>/scans/
POSE_RE = re.compile(r'^[A-Za-z0-9_.-]{1,80}\.json$')   # file directly in the policy root
MAX_STEPS = 20
# policy_bridge.py parameter -> (type it is declared with, default). SetParameters needs the exact type, and every
# key is sent at every step so nothing is left over from the previous policy.
POLICY_KEYS = {
    'task': (str, ''),                  # label for logs and the UI
    'root': (str, None),                # robot: dataset folder for stretch4_goto_pose --root (the start pose)
    'ckpt': (str, None),                # Jetson: checkpoint for stretch4_policy_run --ckpt
    'run_args': (str, ''),              # Jetson: extra stretch4_policy_run arguments, e.g. --fp16 --base
    'max_runtime_sec': (float, 120.0),  # the bridge stops the policy after this
}


def load_policies(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    policies = data.get('policies')
    if not isinstance(policies, dict) or not policies:
        raise ValueError(f'{path}: expected a non-empty "policies:" mapping')
    out = {}
    for name, cfg in policies.items():
        if not NAME_RE.match(str(name)) or not isinstance(cfg, dict):
            raise ValueError(f'{path}: bad policy entry {name!r}')
        unknown = set(cfg) - set(POLICY_KEYS)
        if unknown:
            raise ValueError(f'{path}: policy {name}: unknown keys {sorted(unknown)} '
                             f'(allowed: {sorted(POLICY_KEYS)})')
        missing = [k for k, (_, default) in POLICY_KEYS.items() if default is None and not cfg.get(k)]
        if missing:
            raise ValueError(f'{path}: policy {name}: {", ".join(missing)} required')
        try:
            out[str(name)] = {k: kind(cfg[k]) if k in cfg else default for k, (kind, default) in POLICY_KEYS.items()}
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{path}: policy {name}: {exc}') from None
    return out


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
        out.append({'policy': policy,
                    'start_pose': start_pose,
                    'goal': {k: _number(goal.get(k), f'step {i} goal.{k}') for k in ('x', 'y', 'yaw')},
                    'scan': scan})
    return {'name': name, 'map': map_path, 'steps': out}


def load_mission(path, policies):
    with open(path) as f:
        return validate_mission(yaml.safe_load(f), policies)


def bridge_params(policy_cfg, pose_file=''):
    """All policy_bridge parameters for one step (load_policies already filled in the defaults)."""
    return {**{k: policy_cfg[k] for k in POLICY_KEYS}, 'pose_file': str(pose_file)}
