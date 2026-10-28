#!/usr/bin/env python3
"""Text requests: Claude turns a sentence into a mission built only from what is already set up on this map.

The catalog is every policy config on the map Nav2 runs (a policy with its station: goal, lidar reference, start
pose), every saved mission on that map, the places of their navigate steps and their command steps (word for word).
Claude answers with structured output whose step ids are an enum of that catalog, so it can only combine existing
pieces: it never writes a goal, a pose or a command. The UI saves the
plan as a mission (ask_<time>) and starts it.

    <planner venv>/bin/python planner.py < {"text": ..., "catalog": [...], "model": ...}
        -> {"plan": [ids], "message": "..."} on the last line of stdout; errors: {"error": "..."} and exit code 1

Only ask() needs the anthropic SDK, so this runs with its own venv (docs/installation.md, step 5); the rest is imported
by the UI.
"""

import copy
import json
import os
import sys
from pathlib import Path

KEY_FILE = '~/.config/stretch4_mission/anthropic_api_key'   # used when ANTHROPIC_API_KEY is not set
SYSTEM = """You plan missions for a Stretch 4 mobile manipulator robot. A mission is an ordered list of steps; the \
robot runs them one after another.

You get a request from the operator and a catalog of the steps and missions this robot can run, each with an id and \
a description. Answer with the ids, in order, that do what the request asks. A mission id runs that whole mission. \
Use only ids from the catalog. A policy step drives the robot to its own station first, so it needs no place before \
it.

Before choosing, think through what each step needs and what it leaves done. Some catalog entries say so ("Needs: ...", \
"Leaves: ..."); otherwise use common sense about the task. When a step needs something that no earlier step leaves \
done (an object in the gripper, a door open), add the catalog steps that provide it, in order, before it. Assume \
nothing is done yet unless the request says so. Do not add steps nothing needs.

Plan all or nothing. If any part of the request cannot be done with the catalog, or the request is unclear, answer \
with an empty plan and say in "message" that the mission cannot be completed and which part the robot cannot do: the \
robot never runs part of a request; this includes a step the request needs that the catalog cannot provide. In \
that case also name two or three things from the catalog the robot can do instead, in plain words. Otherwise use "message" to describe the plan in one or two sentences, saying why you added any step the request did \
not name. Write "message" in the language of the request."""


def build_catalog(missions, policies, configs):
    """Policy configs + valid missions, all on one map -> [{id, description, steps}].

    Each config is config:<name>; each mission adds its unique navigate/command steps (and older policy steps written
    inside the mission), then itself. ids are stable (names + step number) so a plan can be read back in the logs.
    """
    def run(policy, where):
        return f"Run policy {policy} ({policies.get(policy, {}).get('task') or policy}) at {where}"

    def notes(c):   # written by the operator: what the policy needs before it runs and what it leaves done
        return ''.join(f'. {k.capitalize()}: {c[k]}' for k in ('needs', 'leaves') if c.get(k))

    catalog = [{'id': f'config:{name}', 'description': run(c['policy'], c.get('label') or name) + notes(c),
                'steps': [{'type': 'policy', 'config': name}]} for name, c in configs.items()]
    seen = set()
    for m in missions:
        parts = []
        for i, step in enumerate(m['steps'], 1):
            kind, where = step['type'], step.get('label') or f"the spot of mission {m['name']} step {i}"
            if step.get('config'):   # already in the catalog as config:<name>
                cfg = configs.get(step['config'], {})
                parts.append(run(step['policy'], step.get('label') or cfg.get('label') or step['config']))
                continue
            if kind == 'policy':
                desc = run(step['policy'], where)
            elif kind == 'navigate':
                desc = f'Go to {where}'
            else:
                desc = f"Run the command {json.dumps(step['command'])}" + (f" ({step['label']})" if step.get('label') else '')
            parts.append(desc)
            key = json.dumps(step, sort_keys=True)
            if key not in seen:   # the same step saved in two missions is one catalog entry
                seen.add(key)
                catalog.append({'id': f"{kind}:{m['name']}:{i}", 'description': desc, 'steps': [step]})
        catalog.append({'id': f"mission:{m['name']}", 'description': f"Whole mission {m['name']}: " + '; then '.join(parts),
                        'steps': m['steps']})
    return catalog


def describe_step(step, policies, configs):
    """A mission step in words a person understands -> {'title', 'where'}: what it does, and where it goes first.

    From the operator's own data (the policy's task in policies.yaml, the config's or step's label), so the Operator
    tab shows real progress, never invented text.
    """
    cfg = configs.get(step.get('config'), {})
    where = step.get('label') or cfg.get('label') or (step.get('config') or '').replace('_', ' ')
    if step['type'] == 'policy':
        task = policies.get(step['policy'], {}).get('task') or step['policy'].replace('_', ' ')
        return {'title': task, 'where': where or 'the station'}
    if step['type'] == 'navigate':
        return {'title': f"Going to {where or 'the next spot'}", 'where': where or 'the next spot'}
    return {'title': step.get('label') or 'Running a command', 'where': ''}


def suggestions(policies, configs, missions, limit=4):
    """Example requests for the Operator tab, from what is set up on the map: the configs' tasks, then missions."""
    task = lambda c: policies.get(c['policy'], {}).get('task') or c['policy'].replace('_', ' ')   # noqa: E731
    tasks = [task(c) for c in configs.values()]
    # the task alone reads best; the place only tells apart one policy set up at two stations
    out = [f"{task(c)} at the {c['label']}" if tasks.count(task(c)) > 1 and c.get('label') else task(c)
           for c in configs.values()]
    out += [f"Run the {m['name'].replace('_', ' ')} mission" for m in missions if not m['name'].startswith('ask_')]
    return list(dict.fromkeys(out))[:limit]


def plan_schema(ids):
    """Structured output: the step ids must come from the catalog."""
    return {'type': 'object',
            'properties': {'plan': {'type': 'array', 'items': {'type': 'string', 'enum': list(ids)}},
                           'message': {'type': 'string'}},
            'required': ['plan', 'message'], 'additionalProperties': False}


def plan_to_mission(plan, catalog, name, map_path):
    """Catalog ids -> a mission dict (validate_mission still checks it before it is saved)."""
    by_id = {c['id']: c for c in catalog}
    unknown = [i for i in plan if i not in by_id]
    if unknown:
        raise ValueError(f'plan uses ids not in the catalog: {unknown}')
    return {'name': name, 'map': map_path, 'steps': [copy.deepcopy(s) for i in plan for s in by_id[i]['steps']]}


def ask(text, catalog, model):
    """One Messages API call -> {'plan': [ids], 'message': str}. Raises RuntimeError with a readable reason."""
    import anthropic
    key_file = Path(os.path.expanduser(KEY_FILE))
    client = (anthropic.Anthropic(api_key=key_file.read_text().strip())
              if not os.environ.get('ANTHROPIC_API_KEY') and key_file.is_file() else anthropic.Anthropic())
    items = '\n'.join(f"- {c['id']}: {c['description']}" for c in catalog)
    try:
        response = client.beta.messages.create(
            model=model,
            max_tokens=16000,
            output_config={'effort': 'high', 'format': {'type': 'json_schema',   # high: chaining needs takes reasoning
                                                          'schema': plan_schema([c['id'] for c in catalog])}},
            betas=['server-side-fallback-2026-07-01'],
            fallbacks='default',   # a safety-classifier decline is retried on Anthropic's recommended model
            system=SYSTEM,
            messages=[{'role': 'user', 'content': f'Catalog:\n{items}\n\nRequest: {text}'}],
        )
    except anthropic.AuthenticationError:
        raise RuntimeError(f'no valid Anthropic API key: put it in {KEY_FILE} (chmod 600) '
                           'or export ANTHROPIC_API_KEY before starting the UI') from None
    except anthropic.RateLimitError:
        raise RuntimeError('rate limited by the Anthropic API; try again in a minute') from None
    except anthropic.APIStatusError as exc:
        raise RuntimeError(f'Anthropic API error {exc.status_code}: {exc.message}') from None
    except anthropic.APIConnectionError:
        raise RuntimeError('cannot reach api.anthropic.com (is the robot online?)') from None
    if response.stop_reason == 'refusal':
        raise RuntimeError('the model declined this request')
    if response.stop_reason == 'max_tokens':
        raise RuntimeError('the answer was cut off; try a shorter request')
    return json.loads(next(b.text for b in response.content if b.type == 'text'))


def main():
    try:
        req = json.load(sys.stdin)
        if not (os.environ.get('ANTHROPIC_API_KEY') or os.environ.get('ANTHROPIC_AUTH_TOKEN')
                or Path(os.path.expanduser(KEY_FILE)).is_file()
                or Path(os.path.expanduser('~/.config/anthropic')).is_dir()):
            raise RuntimeError(f'no Anthropic API key found: create {KEY_FILE} (only the key, chmod 600) '
                               'or export ANTHROPIC_API_KEY before starting the UI')
        out = ask(req['text'], req['catalog'], req['model'])
    except (RuntimeError, ValueError, KeyError, ImportError) as exc:
        print(json.dumps({'error': str(exc)}))
        return 1
    print(json.dumps(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
