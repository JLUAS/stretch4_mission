# Policy steps

A policy step runs one learned policy at its station: the robot drives there, aligns, lifts to the top, moves to the policy start pose, and runs the policy with its runner. Where and how is a [policy config](#policy-configs); the driving and alignment are the same as a [navigate step](navigate-steps.md).

## Policy configs

A policy config is where and how a policy runs on a map: the **Policy configs** tab sets it once, and mission steps and text requests use it by name. Editing a config changes every mission that uses it; deleting one leaves those missions with an error until their step picks another config. Configs are saved in `missions/configs/policy_configs.yaml` (site data, ignored by Git, like the missions):

```yaml
configs:
  cabinet_kitchen:
    map: /home/hello-robot/stretch_user/maps/final/map.yaml
    policy: grab_sponge
    start_pose: start_pose.json         # a pose JSON in the policy's root
    goal: {x: -6.4454, y: -4.345, yaw: -3.1371}
    scan: scan_20261027-113134.json     # lidar reference in missions/scans/
    label: kitchen cabinet              # optional: what text requests call it
    needs: the floor cabinet doors open  # optional: what it needs before it runs (text requests)
    leaves: a sponge held in the gripper  # optional: what it leaves done (text requests)
    max_runtime_sec: 40                 # optional: else the policy's
    run_args: --fp16 --base             # optional: else the policy's
```

To make one: **+ New config**, give it a name and the map, pick the **Policy** (the list shows how it runs, "recorder diffusion" or "LeRobot pi05", and the full command appears below), its **Start pose JSON**, optionally a **Label**, **Needs** and **Leaves** (see [Text requests](operator.md#text-requests)), **Policy run time (s)** and **Run arguments** (empty = the values from `config/policies.yaml`; flags the bridge sets itself are refused). Then drive the robot (gamepad) to **exactly** where that policy's episodes started and click **Use robot pose as goal**: it copies the base pose (x, y, yaw from `map` → `base_link`) and saves a **lidar reference scan** of that spot (`missions/scans/`, median of 1 s of `/scan_filtered`). **Save config**. The tab shows the configs of the map on the map, and the one being edited as a teal arrow. A config's name cannot change, because missions refer to it; make a new one instead.

## Inside a policy step

- **Lift to max** (policy steps; `lift_up_position` 1.2 m, clipped by the driver). Success = measured lift ≥ `lift_min_height` (1.16 m) and settled. **Set `lift_min_height` above every start pose's lift.** The start-pose move (`stretch4_goto_pose` or `stretch4_pose goto`) may interpolate all joints without collision checking, so it should come down from the top.
- **Policy:** the mission sets the step's parameters on `/stretch4_policy_bridge` (all of them, every step, only while no policy runs) and calls `/policy/start`, which moves to the start pose and starts the policy with its [runner](#policy-runners). The mission then polls `/policy/status` until `STOPPED`. "Finished" means the policy **processes** ended; it says nothing about whether the task succeeded. Check that physically.

## Policy runners

Each policy in `config/policies.yaml` says how it runs with `runner`. The UI's policy list shows it after the task, and the step shows the full command. Pick the policy entry with the runner you want; one model can have an entry per runner.

| | `recorder` (default) | `lerobot` |
|---|---|---|
| Models | `stretch4_policy_recorder` diffusion `.ckpt` | LeRobot `pretrained_model` folders: diffusion, pi05, act, ... (`policy_type`) |
| Start pose | `stretch4_goto_pose --root <root> --pose-file <json> --yes` | `stretch4_pose goto <root>/<json> --yes` (lerobot_stretch4 venv) |
| Jetson | `stretch4_policy_run --ckpt <ckpt> <run_args> --yes --duration <max_runtime_sec>` | `stretch4_policy_server --host=0.0.0.0 --port=8080` (a server already listening there is used and left running) |
| Robot | `stretch4_policy_stream` + `stretch4_policy_exec` | `python -m lerobot.async_inference.robot_client --robot.type=stretch4 --robot.mode=control --robot.preprocessing_from=<dataset> --task=<task> --server_address=<jetson>:8080 --pretrained_name_or_path=<ckpt> --policy_type=<policy_type> <run_args>` |
| Stops | At `max_runtime_sec` or when any part exits: Jetson run first (remote `pkill -INT`), then exec, then stream | At `max_runtime_sec` or when either exits: `robot_client` first, then the server if the bridge started it |
| `max_runtime_sec` | Seconds, or `auto` | Seconds (`auto` reads `stretch4_policy_recorder` episodes) |
| Logs | `~/.ros/stretch4_mission/policy_<time>_{goto,stream,exec,run}.log` | `~/.ros/stretch4_mission/policy_<time>_{goto,server,client}.log` |

Every stop is Ctrl-C, then SIGTERM. The policy processes run only inside a step's policy phase, so nothing else commands the robot during navigation.

A `recorder` entry:

```yaml
runtime_margin_sec: 10.0                # for every "auto" policy; a policy can set its own
policies:
  grab_sponge:
    task: Grab the sponge
    root: ~/data/grab_sponge            # on the robot: the dataset, with start_pose.json
    ckpt: ~/policies/grab_sponge.ckpt   # on the Jetson
    run_args: --fp16 --base
    max_runtime_sec: auto               # longest recorded episode + runtime_margin_sec
```

A `lerobot` entry:

```yaml
  floor_cabinet_open:
    runner: lerobot
    task: Floor Cabinet Open                                    # --task (the prompt for pi05)
    root: ~/repos/lerobot_stretch4/policies/floor_cabinet_open  # start_pose.json and other pose files
    ckpt: ~/policies/floor_cabinet_open/pretrained_model        # on the Jetson
    dataset: ~/.cache/huggingface/lerobot/SilentSammy2001/floor_cabinet_open_20261022_105005  # on the robot
    policy_type: diffusion
    run_args: --fps=30 --actions_per_chunk=48 --policy_device=cuda --chunk_size_threshold=0.7 --robot.use_base=true
    max_runtime_sec: 120
```

| Key | Runner | Description |
|---|---|---|
| `runner` | both | `recorder` (default) or `lerobot` |
| `task` | both | Label for the UI and logs; `--task` for `lerobot` |
| `root` | both | Robot folder with `start_pose.json` and other selectable pose files |
| `ckpt` | both | Jetson model: a `.ckpt` file (`recorder`) or a `pretrained_model` folder (`lerobot`) |
| `run_args` | both | Extra `stretch4_policy_run` or `robot_client` arguments; a step can override them |
| `max_runtime_sec` | both | Seconds, or `auto` (`recorder` only); default 120. See [Policy run time](#policy-run-time) |
| `runtime_margin_sec` | `recorder` | Margin `auto` adds for this policy (default: the top-level value) |
| `dataset` | `lerobot` | Robot LeRobot dataset for `--robot.preprocessing_from` |
| `policy_type` | `lerobot` | `--policy_type`: `diffusion`, `pi05`, `act`, ... |

The bridge sets these flags itself, so they are refused in `run_args`: `--ckpt`, `--yes`, `--duration` (`recorder`); `--robot.type`, `--robot.mode`, `--robot.preprocessing_from`, `--task`, `--server_address`, `--pretrained_name_or_path`, `--policy_type` (`lerobot`). The repos, the Jetson host, the server port and the timeouts are in `config/policy_runner.yaml`.

## Adding policies

`scripts/run_ui_robot.sh` runs `sync_policies` before the UI starts. You can also run it by hand:

```bash
ros2 run stretch4_mission sync_policies
```

It adds every policy that can run on this robot and is not in `config/policies.yaml` yet, at the end of the file:

- **`recorder`:** a dataset `~/data/<name>` with a `start_pose.json` whose checkpoint `<name>.ckpt` or `<name>_<anything>.ckpt` is in the Jetson's `~/policies`. The policy is named after the checkpoint and gets `run_args: --fp16 --base` and `max_runtime_sec: auto`.
- **`lerobot`:** a folder `~/repos/lerobot_stretch4/policies/<name>` with a `start_pose.json` whose model `~/policies/<name>/pretrained_model` is on the Jetson. `policy_type` and `--actions_per_chunk` come from the model's `config.json`, `--fps` from the dataset, and `dataset` is the newest `~/.cache/huggingface/lerobot/<user>/<name>[_<anything>]` on the robot. It gets `max_runtime_sec: 120`.

It never changes existing entries (same name or same model) and prints which folders it skipped and why. If the Jetson cannot be reached, the UI still starts with the policies already in the file.

**NOTE**: A synced `lerobot` policy's `dataset` is matched by name only, and its `task` is made from the folder name (pi05 uses it as its prompt). Check both in `config/policies.yaml`.

## Policy run time

Each policy runs for at most `max_runtime_sec`. The bridge then stops it (for `recorder` also through `stretch4_policy_run --duration`), and the mission goes on with the next step.

- `max_runtime_sec: 45` is a fixed limit. Without `max_runtime_sec` a policy gets 120 s.
- `max_runtime_sec: auto` (`recorder` only) is the **longest episode recorded for that policy + `runtime_margin_sec`**. An episode's length is the time `t` of the last sample minus the first in `<root>/episode_*/records.jsonl` (written by `stretch4_policy_recorder`); unreadable or half-written episodes are skipped.
- **Per mission step, in the UI:** the editor's **Policy run time (s)** field. Empty (the default) = the policy's value, shown as the field's hint; a number = this step's own limit, saved in the mission file as the step's `max_runtime_sec`.

**Saved recording lengths (`config/policy_runtimes.json`).** When the UI starts, it measures every policy's episodes and saves the longest one, with the number of episodes, the date and the robot:

```json
{"clean_table": {"episodes": 52, "longest_episode_sec": 68.664, "measured_at": "2026-10-27", "robot": "stretch-se4-4032"}}
```

`auto` uses the dataset when all its episodes are readable, and this file otherwise. To run a `recorder` policy on another robot, copy only `config/policy_runtimes.json` (or commit it) and the start pose JSON files at the top of each dataset (`~/data/<policy>/start_pose.json`, ...); the `episode_*` folders are not needed there. An incomplete copy never replaces a saved length, and recording new episodes updates it at the next UI start. `check_robot` and the editor say where each policy's run time comes from.

The mission works the limits out **when it starts, before the robot moves**, and logs one per policy step (`step 1: grab_sponge: max runtime 36.0 s`). `?` / unknown in the UI means `auto` found no recordings, and the mission would refuse to start. `policy_timeout` (`config/mission.yaml`, 300 s) is how long the mission waits for the bridge to report `STOPPED`, so every limit must be below it; the mission checks this at start.
