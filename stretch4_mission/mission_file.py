"""Load and validate mission and policy-library YAML files."""

import json
import math
import os
import re
import shlex
import socket
import time
from pathlib import Path

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
SCAN_RE = re.compile(r'^[A-Za-z0-9_.-]{1,80}\.json$')   # file in <missions_dir>/scans/
POSE_RE = re.compile(r'^[A-Za-z0-9_.-]{1,80}\.json$')   # file directly in the policy root
MAX_STEPS = 20
STEP_TYPES = ('policy', 'navigate', 'command')   # a step without type is a policy step (older files)
MAX_COMMAND = 1000   # characters of a command step's shell line
MAX_LABEL = 60
MAX_NOTE = 200   # a policy config's needs / leaves
AUTO = 'auto'   # max_runtime_sec: auto -> longest recorded episode of the policy + runtime_margin_sec
RUNNERS = ('recorder', 'lerobot')   # stretch4_policy_recorder (stream/exec + stretch4_policy_run), or LeRobot async
# the bridge sets these itself, per runner: not allowed in run_args
BRIDGE_FLAGS = {'recorder': ('--ckpt', '--yes', '--duration'),
                'lerobot': ('--robot.type', '--robot.mode', '--robot.preprocessing_from', '--task', '--server_address',
                            '--pretrained_name_or_path', '--policy_type')}
MARGIN_KEY = 'runtime_margin_sec'
# Next to policies.yaml: each policy's longest recorded episode, saved by the UI at start. Copy it to another robot
# (or commit it) and "auto" works there without the episode folders.
RUNTIMES_FILE = 'policy_runtimes.json'   # top level (all policies) or per policy; not a bridge parameter
# policy_bridge.py parameter -> (type it is declared with, default). SetParameters needs the exact type, and every
# key is sent at every step so nothing is left over from the previous policy.
POLICY_KEYS = {
    'task': (str, ''),                  # label for logs and the UI
    'root': (str, None),                # robot: folder with the start pose JSON files (recorder: the dataset)
    'ckpt': (str, None),                # Jetson: recorder checkpoint, or lerobot pretrained_model folder
    'run_args': (str, ''),              # extra stretch4_policy_run (recorder) or robot_client (lerobot) arguments
    'max_runtime_sec': (float, 120.0),  # the bridge stops the policy after this (s), or "auto"
    'runner': (str, 'recorder'),        # how it runs: one of RUNNERS
    'dataset': (str, ''),               # lerobot, robot: dataset for --robot.preprocessing_from
    'policy_type': (str, ''),           # lerobot: --policy_type (diffusion, pi05, act, ...)
}


def load_policies(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    policies = data.get('policies')
    if not isinstance(policies, dict) or not policies:
        raise ValueError(f'{path}: expected a non-empty "policies:" mapping')
    margin = _margin(data.get(MARGIN_KEY, 10.0), path)
    saved = read_runtimes(Path(path).with_name(RUNTIMES_FILE))
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
        if policy['runner'] not in RUNNERS:
            raise ValueError(f"{path}: policy {name}: runner must be one of {', '.join(RUNNERS)}")
        if policy['runner'] == 'lerobot':
            if not policy['dataset'] or not policy['policy_type']:
                raise ValueError(f'{path}: policy {name}: runner lerobot needs dataset and policy_type')
            if policy['max_runtime_sec'] == AUTO:   # auto reads stretch4_policy_recorder episodes
                raise ValueError(f'{path}: policy {name}: runner lerobot needs max_runtime_sec in seconds, not auto')
        _run_args(policy['run_args'], f'{path}: policy {name} run_args', policy['runner'])
        policy[MARGIN_KEY] = _margin(cfg[MARGIN_KEY], f'{path}: policy {name}') if MARGIN_KEY in cfg else margin
        policy['recorded'] = saved.get(str(name))   # not a bridge parameter: the saved recording length, or None
        out[str(name)] = policy
    return out


def read_runtimes(path):
    """policy_runtimes.json -> {policy: entry}; a missing file is empty, and malformed entries are left out."""
    try:
        data = json.loads(Path(path).read_text())
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as exc:
        raise ValueError(f'{path}: {exc}') from None
    ok = lambda e: (isinstance(e, dict) and isinstance(e.get('longest_episode_sec'), (int, float))  # noqa: E731
                    and math.isfinite(e['longest_episode_sec']) and e['longest_episode_sec'] > 0)
    return {str(k): v for k, v in data.items() if ok(v)} if isinstance(data, dict) else {}


def save_recorded_runtimes(policies_file):
    """Measure each policy's episodes and save the complete datasets to policy_runtimes.json; -> what changed.

    A dataset that is missing here, has unreadable/cut-off episodes, or has fewer episodes than were saved (a partial
    copy) keeps its saved entry: an incomplete copy never replaces the number measured where it was recorded.
    """
    path = Path(policies_file).with_name(RUNTIMES_FILE)
    saved, changed = read_runtimes(path), []
    for name, cfg in load_policies(policies_file).items():
        secs, total = measure_episodes(cfg['root'])
        if not secs or len(secs) != total or total < saved.get(name, {}).get('episodes', 0):
            continue
        entry = {'longest_episode_sec': round(max(secs), 3), 'episodes': total,
                 'measured_at': time.strftime('%Y-%m-%d'), 'robot': socket.gethostname()}
        old = saved.get(name, {})
        if (old.get('longest_episode_sec'), old.get('episodes')) != (entry['longest_episode_sec'], total):
            saved[name] = entry
            changed.append(f"{name}: {entry['longest_episode_sec']} s ({total} episodes)")
    if changed:
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(saved, indent=2, sort_keys=True) + '\n')
        tmp.replace(path)   # atomic: a mission starting now never reads half a file
    return changed


def _run_args(text, where, runner='recorder'):
    try:
        tokens = shlex.split(text)
    except ValueError as exc:
        raise ValueError(f'{where}: {exc}') from None
    if len(text) > 300:
        raise ValueError(f'{where}: too long (300 characters max)')
    managed = sorted({t.split('=', 1)[0] for t in tokens} & set(BRIDGE_FLAGS[runner]))
    if managed:
        raise ValueError(f"{where}: {', '.join(managed)} are set by the bridge; remove them")
    return text


def _margin(value, where):
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = math.nan
    if not math.isfinite(value) or value < 0:
        raise ValueError(f'{where}: {MARGIN_KEY} must be a number >= 0')
    return value


def episode_sec(records):
    """One episode's length: last minus first sample time "t" in its records.jsonl; None if unreadable or cut off."""
    try:
        with open(records, 'rb') as f:
            first = json.loads(f.readline())['t']
            f.seek(max(0, f.seek(0, os.SEEK_END) - 65536))   # only the tail: the files are ~1 MB
            last = json.loads([line for line in f.read().splitlines() if line.strip()][-1])['t']
        return float(last) - float(first)
    except (OSError, ValueError, KeyError, IndexError, TypeError):
        return None


def measure_episodes(root):
    """-> (lengths of the readable episodes, number of episodes) of a dataset."""
    files = list(Path(os.path.expanduser(root)).glob('episode_*/records.jsonl'))
    return [s for s in map(episode_sec, files) if s is not None], len(files)


def _no_episodes(root, total):
    why = (f'{total} found, all unreadable or cut off: copy the dataset again' if total
           else 'no episode_*/records.jsonl')
    return ValueError(f'max_runtime_sec auto: no readable recorded episodes in {root} ({why}) '
                      f'and none saved in {RUNTIMES_FILE}')


def longest_episode_sec(root):
    """Longest episode recorded in a stretch4_policy_recorder dataset (unreadable or cut-off episodes do not count)."""
    secs, total = measure_episodes(root)
    if not secs:
        raise _no_episodes(root, total)
    return max(secs)


def recorded_runtime(policy_cfg):
    """-> (longest recorded episode in s, where that number comes from).

    From the dataset when all its episodes are readable and it is not a partial copy; otherwise the length saved in
    policy_runtimes.json (the episodes were deleted, copied incompletely, or only some were copied); otherwise
    whatever episodes are readable.
    """
    secs, total = measure_episodes(policy_cfg['root'])
    saved = policy_cfg.get('recorded')
    partial_copy = saved is not None and saved.get('episodes', 0) > total
    if secs and len(secs) == total and not partial_copy:
        return max(secs), f'{total} episodes'
    if saved:
        return float(saved['longest_episode_sec']), (f"saved in {RUNTIMES_FILE}, {saved.get('measured_at', '?')} "
                                                     f"on {saved.get('robot', '?')}")
    if secs:
        return max(secs), f'only {len(secs)}/{total} episodes readable'
    raise _no_episodes(policy_cfg['root'], total)


def _number(value, what):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{what}: not a number') from None
    if not math.isfinite(value) or abs(value) > 1000.0:
        raise ValueError(f'{what}: out of range')
    return value


def validate_mission(data, policies, maps=None, configs=None):
    """Return a normalized mission dict or raise ValueError. maps: allowed map paths (UI only).

    configs: the policy configs ({name: config}, load_configs). A policy step names one (config:) and comes back
    resolved: its policy, goal, lidar reference and start pose, plus config. stored() turns it back into the name.
    Older files with the policy written inside the step still load.
    """
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
        kind = str(step.get('type') or 'policy')
        if kind not in STEP_TYPES:
            raise ValueError(f"step {i}: type must be one of {', '.join(STEP_TYPES)}")
        label = _label(step, f'step {i}')
        if kind == 'command':
            out.append({**_command_step(step, i), **label})
        elif kind == 'policy' and step.get('config'):
            config = str(step['config'])
            cfg = (configs or {}).get(config)
            if cfg is None:
                raise ValueError(f'step {i}: unknown policy config {config!r} (Policy configs tab)')
            if os.path.realpath(cfg['map']) != os.path.realpath(map_path):
                raise ValueError(f"step {i}: policy config {config} is on {Path(cfg['map']).name}, "
                                 f'the mission on {Path(map_path).name}')
            out.append({'type': 'policy', 'config': config,
                        **{k: v for k, v in cfg.items() if k not in ('map', 'label', 'needs', 'leaves')}, **label})
        else:
            entry = {'type': kind, **_place(step, f'step {i}')}
            if kind == 'policy':   # an older step with the policy written inside it
                entry.update(_policy_run(step, f'step {i}', policies))
            out.append({**entry, **label})
    return {'name': name, 'map': map_path, 'steps': out}


def stored(mission):
    """A validated mission as it is saved: a policy config step keeps only its name (and its own label)."""
    keep = ('type', 'config', 'label')
    return {**mission, 'steps': [{k: v for k, v in s.items() if k in keep} if s.get('config') else s
                                 for s in mission['steps']]}


def _place(step, where):
    """A goal {x, y, yaw} on the map plus an optional lidar reference scan (missions/scans/)."""
    goal = step.get('goal')
    if not isinstance(goal, dict):
        raise ValueError(f'{where}: goal must be {{x, y, yaw}}')
    scan = str(step.get('scan') or '')
    if scan and not SCAN_RE.match(scan):
        raise ValueError(f'{where}: bad reference scan name {scan!r}')
    return {'goal': {k: _number(goal.get(k), f'{where} goal.{k}') for k in ('x', 'y', 'yaw')}, 'scan': scan}


def _policy_run(src, where, policies):
    """The policy, its start pose, and optional own run time / run arguments (else the policy's from policies.yaml)."""
    policy = str(src.get('policy', ''))
    if policy not in policies:
        raise ValueError(f'{where}: unknown policy {policy!r}')
    start_pose = str(src.get('start_pose') or 'start_pose.json')
    if not POSE_RE.match(start_pose):
        raise ValueError(f'{where}: bad start pose name {start_pose!r}')
    out = {'policy': policy, 'start_pose': start_pose}
    runtime = src.get('max_runtime_sec')
    if runtime not in (None, ''):
        out['max_runtime_sec'] = _number(runtime, f'{where} max_runtime_sec')
        if out['max_runtime_sec'] <= 0:
            raise ValueError(f'{where} max_runtime_sec: must be > 0 (leave it empty for the policy default)')
    run_args = str(src.get('run_args') or '').strip()
    if run_args:
        out['run_args'] = _run_args(run_args, f'{where} run_args', policies[policy].get('runner', 'recorder'))
    return out


def _label(step, where):
    """Optional name ("kitchen", "wave"): what text requests (planner.py) call a step or config. -> {} or {'label': ...}"""
    label = str(step.get('label') or '').strip()
    if len(label) > MAX_LABEL or any(c in label for c in '\n\r\0'):
        raise ValueError(f'{where} label: one line of up to {MAX_LABEL} characters')
    return {'label': label} if label else {}


CONFIGS_FILE = 'configs/policy_configs.yaml'   # under the missions folder, like scans/: site data, like the missions
CONFIGS_HEADER = """# Policy configs: where and how each policy runs on a map (goal, lidar reference, start pose, run settings).
# Written by the UI (Policy configs tab). Mission steps name them (config:), so editing one updates every mission.
"""


def validate_config(name, cfg, policies, maps=None):
    """One policy config -> {map, policy, start_pose, goal, scan, [label, max_runtime_sec, run_args, needs, leaves]}."""
    where = f'policy config {name}'
    if not NAME_RE.match(str(name)) or not isinstance(cfg, dict):
        raise ValueError(f'{where}: name 1-64 letters, digits, "_" or "-", and a mapping')
    map_path = str(cfg.get('map', ''))
    if not map_path.endswith('.yaml') or (maps is not None and map_path not in maps):
        raise ValueError(f'{where}: map: choose one of the maps in the list ({map_path!r} is not)')
    notes = {}
    for key in ('needs', 'leaves'):   # what the policy needs before it runs / leaves done after: text requests chain on them
        text = str(cfg.get(key) or '').strip()
        if len(text) > MAX_NOTE or any(c in text for c in '\n\r\0'):
            raise ValueError(f'{where} {key}: one line of up to {MAX_NOTE} characters')
        if text:
            notes[key] = text
    return {'map': map_path, **_policy_run(cfg, where, policies), **_place(cfg, where), **_label(cfg, where), **notes}


def load_configs(path, policies, maps=None):
    """policy_configs.yaml -> ({name: config}, {name: error}). No file = none; a bad entry is reported, not fatal."""
    path = Path(path)
    data = yaml.safe_load(path.read_text()) if path.is_file() else {}
    entries = (data or {}).get('configs') or {} if isinstance(data or {}, dict) else None
    if not isinstance(entries, dict):
        raise ValueError(f'{path}: expected "configs:" with one entry per policy config')
    good, bad = {}, {}
    for name, cfg in entries.items():
        try:
            good[str(name)] = validate_config(str(name), cfg, policies, maps)
        except ValueError as exc:
            bad[str(name)] = str(exc)
    return good, bad


def write_config(path, name, cfg):
    """Set (cfg: a validated config) or delete (cfg None) one entry, keeping the others as written; atomic."""
    path = Path(path)
    data = (yaml.safe_load(path.read_text()) if path.is_file() else None) or {}
    entries = dict(data.get('configs') or {})
    if cfg is None:
        entries.pop(name, None)
    else:
        entries[name] = cfg
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(CONFIGS_HEADER + yaml.safe_dump({'configs': entries}, sort_keys=False))
    tmp.replace(path)   # a running mission never reads a half-written file


def _command_step(step, i):
    """A shell line run on the robot (bash, home folder); exit code 0 = success."""
    command = str(step.get('command') or '').strip()
    if not 1 <= len(command) <= MAX_COMMAND or any(c in command for c in '\n\r\0'):
        raise ValueError(f'step {i} command: one line of 1-{MAX_COMMAND} characters')
    entry = {'type': 'command', 'command': command}
    timeout = step.get('timeout_sec')
    if timeout not in (None, ''):   # without it, command_timeout (config/mission.yaml) applies
        entry['timeout_sec'] = _number(timeout, f'step {i} timeout_sec')
        if entry['timeout_sec'] <= 0:
            raise ValueError(f'step {i} timeout_sec: must be > 0 (leave it empty for the default)')
    return entry


def parse_goal(text):
    """'x,y,yaw' in the map frame -> {'x', 'y', 'yaw'}: the target of a single go-to-pose run."""
    parts = str(text).split(',')
    if len(parts) != 3:
        raise ValueError('goal: expected "x,y,yaw"')
    return {k: _number(v, f'goal.{k}') for k, v in zip(('x', 'y', 'yaw'), parts)}


def load_mission(path, policies):
    """A mission file, its policy config steps resolved from policy_configs.yaml next to it."""
    configs, _ = load_configs(Path(path).parent / CONFIGS_FILE, policies)
    with open(path) as f:
        return validate_mission(yaml.safe_load(f), policies, configs=configs)


def bridge_params(policy_cfg, pose_file='', max_runtime_sec=None, run_args=None):
    """All policy_bridge parameters for one step (load_policies already filled in the defaults).

    max_runtime_sec, run_args: the step's own, if the mission sets them. Otherwise the policy's; max_runtime_sec
    "auto" becomes the policy's longest recorded episode + runtime_margin_sec.
    """
    params = {**{k: policy_cfg[k] for k in POLICY_KEYS}, 'pose_file': str(pose_file)}
    if run_args is not None:
        params['run_args'] = run_args
    if max_runtime_sec is not None:
        params['max_runtime_sec'] = float(max_runtime_sec)
    elif params['max_runtime_sec'] == AUTO:
        params['max_runtime_sec'] = recorded_runtime(policy_cfg)[0] + policy_cfg['runtime_margin_sec']
    return params
