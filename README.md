# Overview

*stretch4_mission* runs multi-step missions on the Stretch 4 mobile manipulator (ROS 2 Jazzy, `rmw_zenoh_cpp`) from a web UI. A mission drives the robot with Nav2 to each station, aligns it to a saved lidar reference, and runs a learned policy, a shell command, or nothing there. Missions are made in the editor, or from a sentence: Claude builds one from the steps of the saved missions (see [Text requests](#text-requests)). It is an independent project built on the Stretch 4 ROS 2 packages (`stretch_core`, `stretch_nav2`); it is not maintained by Hello Robot Inc.

![Stretch 4 Missions web UI](ui.png)

A **mission** is a map plus ordered **steps**. The three step types can be combined in any order, for example "clean the table": open cabinet → pick sponge → clean table. Every step starts with the same arm retract: **lift to 0.5 m and retract the arm (the order depends on the lift height) → stow the wrist**. Then:

| Step type | What it does after the arm retract |
|---|---|
| **Policy** | Check pose → Nav2 to the step goal if needed → verify arrival and a stopped base (lidar alignment when the step has a reference) → lift to the top → `/policy/start` (the start pose, then the policy with its [runner](#policy-runners)) → wait for `STOPPED` |
| **Navigate only** | Check pose → Nav2 to the step goal if needed → verify arrival and a stopped base (lidar alignment when the step has a reference) |
| **Command** | Run one shell line on the robot where it is (no driving), for example `python3 ~/scripts/wave.py`. Exit code 0 = success. See [Command steps](#command-steps). |

### Repository layout

| Resource | Description |
|---|---|
| [config/](config/) | Mission tuning (`mission.yaml`), the policy library (`policies.yaml`), policy runner settings (`policy_runner.yaml`), saved recording lengths (`policy_runtimes.json`) |
| [launch/](launch/) | `navigation.launch.py` (driver, lidars, Nav2, policy bridge) and `mission.launch.py` (one mission) |
| [missions/](missions/) | Missions, policy configs (`configs/`) and lidar reference scans (`scans/`) written by the UI (ignored by Git) |
| [scripts/](scripts/) | `run_ui_robot.sh`, the operator entry point |
| [stretch4_mission/](stretch4_mission/) | Python package: the nodes, the web UI (`web/index.html`) and the command-line tools |
| [test/](test/) | Pytest suite |

### Components it builds on

| Piece | Existing component |
|---|---|
| Navigation, AMCL, obstacle avoidance | `stretch_nav2` `navigation_mppi.launch.py` (also starts `stretch_driver` in `navigation` mode, the dual Hesai lidars and the footprint) |
| Global localization | AMCL `reinitialize_global_localization` + `request_nomotion_update`, Nav2 `spin` action |
| Goal | Nav2 `NavigateToPose` through `nav2_simple_commander.BasicNavigator` |
| Pose / stationary | TF2 `map` → `base_link`, `/wheel_odom` |
| Arm retract, lift up | `stretch_driver`'s `/follow_joint_trajectory`, one joint at a time |
| Policies | `policy_bridge` (this package) behind `/policy/start`, `/policy/stop` and `/policy/status`, with two runners: `stretch4_policy_recorder` or `lerobot_stretch4` (see [Policy runners](#policy-runners)) |

## Quickstart

On the robot, build the package, check the robot, then start the UI:

```bash
cd ~/ament_ws/src
git clone git@github.com:JLUAS/stretch4_mission.git
cd ~/ament_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stretch4_mission
source install/setup.bash
ros2 run stretch4_mission check_robot
~/ament_ws/src/stretch4_mission/scripts/run_ui_robot.sh
```

From your computer, open an SSH tunnel and then http://localhost:8090:

```bash
ssh -N -L 8090:localhost:8090 hello-robot@<robot-ip>
```

**NOTE**: Home the robot, clear the area and keep a person with the physical stop next to the robot before you start a mission. The robot spins in place to localize and lifts to the top before every policy.

## Installation

1. **Policy runners.** Install the tools of the runners you use, each with its `.venv`, on the robot and on the Jetson: `stretch4_policy_recorder` in `~/repos/stretch4_policy_recorder` for the `recorder` runner, `lerobot_stretch4` in `~/repos/lerobot_stretch4` for the `lerobot` runner. The robot must log in to the Jetson with an ssh key (no password): `ssh -o BatchMode=yes jetson1@192.168.1.101 true` on the robot must succeed. Settings: `config/policy_runner.yaml`.
2. **Each policy needs** a folder on the robot with `start_pose.json` and, optionally, more pose JSON files beside it (each mission step selects one), and its model on the Jetson. For the `recorder` runner the folder is the dataset (`~/data/<name>`); `stretch4_goto_pose --root <dataset> --pose-file <json> --show` prints a pose. For the `lerobot` runner it is `~/repos/lerobot_stretch4/policies/<name>`, and the robot also needs the LeRobot dataset.
3. **Add each policy** to `config/policies.yaml` (templates at the bottom of the file). `scripts/run_ui_robot.sh` does this for you: before the UI starts it runs `sync_policies` (see [Adding policies](#adding-policies)).
4. **Build** as in the [Quickstart](#quickstart). The install is a copy, so rebuild after code changes. Policies, missions and `config/mission.yaml` are read from `~/ament_ws/src/stretch4_mission/`, so editing them needs no rebuild (`mission.yaml` only when started through `ros2 launch` without `mission_config:=`).
5. **Text requests (optional)**, for [Text requests](#text-requests). On the robot, with internet:

   ```bash
   # the Anthropic SDK, in its own venv (the ROS Python stays untouched)
   python3 -m venv ~/.venvs/stretch4_planner
   ~/.venvs/stretch4_planner/bin/pip install anthropic

   # the API key: only the key, on one line, readable only by the robot's user
   mkdir -p ~/.config/stretch4_mission
   nano ~/.config/stretch4_mission/anthropic_api_key
   chmod 600 ~/.config/stretch4_mission/anthropic_api_key
   ```

   Create the key in the Claude Console (https://platform.claude.com, **API keys**); the requests are billed to that account. The key file is outside the repository, so it is never committed or copied with the code; never put the key in `config/` or `missions/`. Instead of the file, `ANTHROPIC_API_KEY` exported in the UI's environment also works. The robot needs to reach `api.anthropic.com` (HTTPS). Restart the UI afterwards, and check: `ros2 run stretch4_mission check_robot` shows the venv and the key under **Text requests**.
6. **On another robot**, nothing changes for the robot itself: `scripts/run_ui_robot.sh` reads its ID from `/etc/hello-robot/hello-robot.conf` (`HELLO_FLEET_ID=stretch-se4-…`), which every Stretch has, so the driver and lidars load that robot's calibration. Do check, on that robot:
   - the start pose folders and datasets: `root:` (and `dataset:` for `lerobot`) of each policy in `config/policies.yaml`;
   - the Jetson: `jetson_host` in `config/policy_runner.yaml`, and the models (`ckpt:`) on it;
   - the map: the script preselects `maps/test.yaml`; without it the UI shows the first map in `~/stretch_user/maps`.

## Checking the robot

```bash
source /opt/ros/jazzy/setup.bash && source ~/ament_ws/install/setup.bash
ros2 run stretch4_mission check_robot
```

`check_robot` is read-only: it starts, stops and moves nothing. Each line is ✓ (fine), ! (warning) or ✗ (problem), and every ! and ✗ says how to fix it. At the end it prints `READY` (exit code 0) or `NOT READY: <n> problem(s)` (exit code 1).

| Section | What it checks |
|---|---|
| Environment | ROS Jazzy sourced, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, the zenoh router, `HELLO_FLEET_PATH`, the package source folder |
| Robot | UI port 8090 free (no UI already running), `stretch_body_server` running, no `stretch4_policy_stream`/`exec` started by hand (they block the bridge's), homed / runstop / battery (through stretch_nav_webapp's readiness probe, when installed) |
| Maps | Maps in `$HELLO_FLEET_PATH/maps` (also `<folder>/map.yaml` from stretch_nav_webapp) with their images, the default `test.yaml` |
| Policies | `config/policies.yaml` loads; per policy: start pose folder, `start_pose.json`, the LeRobot dataset (`lerobot`), run time (for `auto`: how many episodes are readable) |
| Missions | Every `missions/*.yaml` is valid for the current policies and maps, and its lidar references exist |
| Policy runner | The robot tools of each runner in use, ssh to the Jetson without a password, `stretch4_policy_run` / `stretch4_policy_server` and every policy's model on the Jetson |
| Text requests | The planner venv with the `anthropic` SDK and an API key (warnings only: the feature is optional) |

Options: `--src <folder>` when the package is not in `~/ament_ws/src/stretch4_mission`, `--maps <folder>`, `--no-jetson` to skip the Jetson.

## Using the web UI

Start it on the robot (the script loads the same environment as `~/.bashrc`, so it also works from a plain `ssh` command):

```bash
~/ament_ws/src/stretch4_mission/scripts/run_ui_robot.sh
```

The script uses this robot's ID, adds new policies to `config/policies.yaml` (`sync_policies`), preselects `$HELLO_FLEET_PATH/maps/test.yaml` and turns RViz off. The UI listens only on the robot itself (`127.0.0.1:8090`); reach it through the SSH tunnel from the [Quickstart](#quickstart). Extra arguments go to the node: for example `-p host:=0.0.0.0` serves the whole network at `http://<robot-ip>:8090`.

**NOTE**: The UI has no login. Anyone who can reach its port can move the robot and, with command steps, run any shell command on it as the robot's user. Keep the default `host` (`127.0.0.1`) and use an SSH tunnel, or use it only on a trusted lab network.

1. **Robot:** choose the map, then **Start all**, or start each part with its own button: **Robot driver**, **Lidars**, **Nav2** (on the selected map) and **Policy bridge**. Together they are what `stretch_nav2`'s `navigation_mppi.launch.py` starts (same included launch files and arguments), plus the bridge. Each row shows whether that part really works, not just whether its process exists: the driver's `/follow_joint_trajectory` action and `/joint_states`, fresh `/scan_filtered`, Nav2's `navigate_to_pose`, the bridge's `/policy/status`. Green = ready, yellow = started but not responding yet. A part already running outside the UI shows as "started from a terminal", and the UI does not start a second copy. Stop buttons send Ctrl-C and are disabled while a mission runs. Each part has its own log (selector at the bottom).

   ![Step 1: choose a map and start the robot services](docs/ui/01-start-services.png)

2. **Localize (global):** AMCL spreads its guesses over the whole map, and the robot rotates in place (Nav2 `spin`, collision-checked, up to 3 turns) until the estimate converges (σxy ≤ 0.25 m, σyaw ≤ 0.30 rad). Check that the blue robot on the map is where the real robot is; in rooms that look alike AMCL can converge to the wrong place.

   ![Step 2: global localization controls and status](docs/ui/02-localize.png)

3. **Set up the policies** in the **Policy configs** tab (top of the page), once per policy and station. See [Policy configs](#policy-configs).
4. **Create a mission** in the **Missions** tab:
   1. Click **+ New mission**, give it a name and pick the map.
   2. Click **+ Add step** and pick its **Step type**: *Policy*, *Navigate only* or *Command on the robot*. A policy step picks a **policy config**; its station, start pose and run settings come from that config.
   3. For navigate steps, drive the robot (gamepad) to where it should stop, then click **Use robot pose as goal** (it captures a lidar reference, as for a config). Goals appear as numbered arrows on the map.
   4. For command steps, type the **Command** (one line) and, optionally, a **Timeout (s)**. See [Command steps](#command-steps).
   5. Optionally give steps a **Label**, the name [text requests](#text-requests) use for them. Reorder steps with ↑ ↓, then **Save mission**.

   ![Step 3: configure policies, start poses, and navigation goals](docs/ui/03-edit-mission.png)

5. **Start** the mission, or type a [text request](#text-requests) in **Ask & run**. The state machine shows every step and phase live: pending, active (blue), done (green), skipped (dashed, for example navigation when already in place) and failed (red). Each step shows only the phases of its type. **Stop mission** sends Ctrl-C, and the mission cancels whatever it was doing. Navigation keeps running, so you can start another mission.

   ![Step 4: select a mission and follow its state machine](docs/ui/04-run-mission.png)

A mission only starts when Nav2 (started from the UI) runs on **its** map, the driver, lidars and bridge are ready, and the robot is localized. The UI says which part is missing.

**Go to pose** (under the map) drives the robot to any spot without a mission: press on the map where it should go and drag toward where it should face (a teal arrow), then click **Go to pose**. The robot stows the arm with stretch_nav_webapp's stow (the robot's own stow pose; the robot must be homed), then Nav2 drives there through the webapp's NavigateToPose client. It shows in the state machine as a one-step navigate mission (alignment skipped), and **Stop mission** stops it. It needs the same as a mission: Nav2 on the map shown, every part ready, the robot localized. From a terminal: `ros2 launch stretch4_mission mission.launch.py navigation:=false goal:=x,y,yaw`.

## Missions

Missions are saved by the UI in `missions/<name>.yaml` (`name`, `map`, `steps`). Each step has a `type` (missing = `policy`, as in older files):

| Type | Keys |
|---|---|
| `policy` | `config`: the name of a [policy config](#policy-configs) on the mission's map. Older files may instead hold `policy`, `start_pose`, `goal`, `scan`, `max_runtime_sec` and `run_args` in the step; they still run, and the editor converts them into a config. |
| `navigate` | `goal: {x, y, yaw}`, `scan` |
| `command` | `command` (one line), optional `timeout_sec` |

Every step can also have a `label` (one line, up to 60 characters), the name [text requests](#text-requests) use for it: "kitchen" for a navigate step, "wave" for a command step.

### Policy configs

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

To make one: **+ New config**, give it a name and the map, pick the **Policy** (the list shows how it runs, "recorder diffusion" or "LeRobot pi05", and the full command appears below), its **Start pose JSON**, optionally a **Label**, **Needs** and **Leaves** (see [Text requests](#text-requests)), **Policy run time (s)** and **Run arguments** (empty = the values from `config/policies.yaml`; flags the bridge sets itself are refused). Then drive the robot (gamepad) to **exactly** where that policy's episodes started and click **Use robot pose as goal**: it copies the base pose (x, y, yaw from `map` → `base_link`) and saves a **lidar reference scan** of that spot (`missions/scans/`, median of 1 s of `/scan_filtered`). **Save config**. The tab shows the configs of the map on the map, and the one being edited as a teal arrow. A config's name cannot change, because missions refer to it; make a new one instead.

### Between and inside steps

- **Arm and lift moves are judged on the measured joint**, not on the driver's action result. `stretch_driver` only finishes a trajectory after the joint has *started* moving, so a goal at the current position (arm already retracted, lift already at its top) would never finish. A joint already in place is skipped ("already there"), a move counts as done once the joint is in place and still (a goal the driver keeps tracking is cancelled), and a driver error fails the step at once.
- **At the start of every step (policy, navigate and command), the arm retracts and the lift moves to 0.5 m.** With the lift below 0.5 m it lifts first, then retracts; at or above 0.5 m it retracts first, then lowers. **Then the wrist moves to the SG4 stow orientation** (yaw 3.14, roll 0, pitch 0). The gripper is not commanded, so a held object stays held. `retract_arm_before_navigation: false` in `config/mission.yaml` turns this off for every step type.
- **Navigation:** before each goal, the mission tightens Nav2's goal checker at runtime to 0.10 m / 0.08 rad (`stretch_nav2` accepts 0.25 m / 0.34 rad). After success it waits for the base to stop, then checks 0.25 m / 0.15 rad itself.
- **Align (lidar):** AMCL alone is good to about 5-10 cm and can jump after the robot stops, too coarse for a policy trained from a fixed spot. For steps with a lidar reference, the robot waits until it is still, checks AMCL is within 0.40 m / 0.50 rad, then matches the live scan to the reference (ICP) and corrects with slow omni-base moves on odometry, repeating until within 1 cm / 0.01 rad (`align_*` in `config/mission.yaml`). It fails without starting the policy if the scans match poorly (surroundings changed, < 60 % of points), the correction is too large, or something comes within 0.26 m of the base center. Steps without a reference use the AMCL check (0.25 m / 0.15 rad) instead. Measured on the robot: two captures of the same spot agree within 0.7 mm / 0.01 deg (96 % match).
- **Lift to max** (policy steps; `lift_up_position` 1.2 m, clipped by the driver). Success = measured lift ≥ `lift_min_height` (1.16 m) and settled. **Set `lift_min_height` above every start pose's lift.** The start-pose move (`stretch4_goto_pose` or `stretch4_pose goto`) may interpolate all joints without collision checking, so it should come down from the top.
- **Policy:** the mission sets the step's parameters on `/stretch4_policy_bridge` (all of them, every step, only while no policy runs) and calls `/policy/start`, which moves to the start pose and starts the policy with its [runner](#policy-runners). The mission then polls `/policy/status` until `STOPPED`. "Finished" means the policy **processes** ended; it says nothing about whether the task succeeded. Check that physically.
- **Command:** see [Command steps](#command-steps).

**Tip**: Keep the area around each station as it was when its lidar reference was captured, and capture it again if furniture moves.

### Command steps

A command step runs one line with `bash -c` on the robot, as the user that started the UI, in the home folder, with the UI's environment (ROS sourced, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`), so a Python script can use `rclpy` and `stretch_driver`. It runs where the robot is, after the arm retract; put a **Navigate only** step before it to move first. Exit code 0 = the step passed; any other code fails the mission. After `timeout_sec` (or `command_timeout` in `config/mission.yaml`, default 300 s; at most 1000 s) the mission sends Ctrl-C to the command's process group, waits 5 s, then kills what is left. **Stop mission** does the same. Output: `~/.ros/stretch4_mission/command_<time>.log`.

**NOTE**: `stretch_driver` stays in navigation mode and nothing else commands the robot while the command runs; the script is responsible for what it moves.

### Text requests

Type a request in **Ask & run** (above the mission list), for example "go to the kitchen, then open the cabinet". Claude (`claude-opus-5-5`) builds a mission from the policy configs and saved missions on the map Nav2 runs, the UI saves it as `missions/ask_<time>.yaml`, and starts it at once. It shows in the mission list, so it can be edited, run again or deleted like any other.

Claude can only combine what is already set up: it answers with structured output whose step ids are an enum of a catalog built from the policy configs and saved missions on that map:

| Catalog entry | From |
|---|---|
| `config:<name>` | Every policy config on the map: the robot drives to its station, aligns, and runs the policy. A policy with no config cannot run. |
| `mission:<name>` | A whole saved mission |
| `navigate:<mission>:<n>` | A place: the goal of a navigate step |
| `command:<mission>:<n>` | A command step, word for word |
| `policy:<mission>:<n>` | An older policy step written inside a mission (until it is converted into a config) |

It never writes a goal, a pose or a command. Any request is accepted, in any language; planning is all or nothing: when any part of the request cannot be done with the saved steps, nothing runs and the UI shows that the mission cannot be completed and which part the robot cannot do. Give configs and steps a `label` ("kitchen cabinet", "wave"): it is the name Claude matches the request against.

Claude also works out what the request needs but does not say. Before planning it thinks through what each step needs and what it leaves done, and adds the steps that provide a missing need, in order: "clean the table" becomes open the floor cabinet → grab the sponge → clean the table, and the reply says why each extra step was added. It assumes nothing is done yet unless the request says so ("the cabinet is already open"). Common sense covers obvious needs; for the ones that depend on the site (where the sponge is kept), write them in each config's **Needs** and **Leaves**, so the chain comes from you and not from a guess:

| Config | Needs | Leaves |
|---|---|---|
| `open_floor_cabinet` | | the floor cabinet doors open |
| `grab_sponge` | the floor cabinet doors open | a sponge held in the gripper |
| `clean_table` | a sponge held in the gripper | the table clean |

When a needed step does not exist as a config, nothing runs and the reply says what is missing. The robot must be ready to run a mission (Nav2 on the map, every part ready, localized) before a request is sent; each request is one Messages API call, billed to the key's account.

**NOTE**: The plan runs without a confirmation, including command steps chosen from the saved missions. Save in missions only commands you want a text request to be able to run, and keep the physical stop within reach.

Setup: the planner venv and the API key, once per robot ([Installation](#installation), step 5).

| Message under **Ask & run** | Fix |
|---|---|
| `…/.venvs/stretch4_planner/bin/python not found` | The planner venv is missing: run the two venv commands of [Installation](#installation), step 5. No UI restart needed. |
| `no Anthropic API key found` / `no valid Anthropic API key` | Put the key in `~/.config/stretch4_mission/anthropic_api_key` (`chmod 600`), or check it in the Claude Console. |
| `cannot reach api.anthropic.com` | The robot has no internet: text requests need HTTPS to `api.anthropic.com`. |
| `Nothing set up on <map>` | No policy configs and no missions on the map Nav2 runs: create configs in the **Policy configs** tab. Older missions with the policy written inside the step still count; their **Convert to a policy config** button (in the mission editor) turns each step into a config. |
| `Start Nav2 on the map first` / `Not ready: …` / `Localize the robot first` | A request is only sent when a mission could start right away; do steps 1-2 of [Using the web UI](#using-the-web-ui). |
| The mission cannot be completed … | Not an error: the request needs something no config, place or command on this map provides (the reply says what). Add that config, or ask for less. |
| `rate limited by the Anthropic API` | Wait a minute and ask again. | `stretch4_mission/planner.py` is the only part that talks to the model (input: the request and the catalog; output: one JSON line `{"plan", "message"}`), so moving to another model changes only that file. The UI parameters `planner_python` and `planner_model` select the venv and the model.

## Policies

### Policy runners

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

### Adding policies

`scripts/run_ui_robot.sh` runs `sync_policies` before the UI starts. You can also run it by hand:

```bash
ros2 run stretch4_mission sync_policies
```

It adds every policy that can run on this robot and is not in `config/policies.yaml` yet, at the end of the file:

- **`recorder`:** a dataset `~/data/<name>` with a `start_pose.json` whose checkpoint `<name>.ckpt` or `<name>_<anything>.ckpt` is in the Jetson's `~/policies`. The policy is named after the checkpoint and gets `run_args: --fp16 --base` and `max_runtime_sec: auto`.
- **`lerobot`:** a folder `~/repos/lerobot_stretch4/policies/<name>` with a `start_pose.json` whose model `~/policies/<name>/pretrained_model` is on the Jetson. `policy_type` and `--actions_per_chunk` come from the model's `config.json`, `--fps` from the dataset, and `dataset` is the newest `~/.cache/huggingface/lerobot/<user>/<name>[_<anything>]` on the robot. It gets `max_runtime_sec: 120`.

It never changes existing entries (same name or same model) and prints which folders it skipped and why. If the Jetson cannot be reached, the UI still starts with the policies already in the file.

**NOTE**: A synced `lerobot` policy's `dataset` is matched by name only, and its `task` is made from the folder name (pi05 uses it as its prompt). Check both in `config/policies.yaml`.

### Policy run time

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

## Nodes

### mission_controller

Runs one mission (or one go-to-pose goal) and exits. Started by the UI or by `mission.launch.py`; its parameters are in `config/mission.yaml`, where each one is commented.

| Parameter | Type | Description |
|---|---|---|
| `mission_file` | string | Mission YAML to run |
| `goal` | string | `x,y,yaw`: go to this pose instead of running a mission |
| `policies_file` | string | Policy library (default `~/ament_ws/src/stretch4_mission/config/policies.yaml`) |
| `xy_tolerance`, `yaw_tolerance` | double | Final pose check after Nav2 (m, rad) |
| `nav2_xy_goal_tolerance`, `nav2_yaw_goal_tolerance` | double | Nav2 goal checker, set at runtime before each goal |
| `align_*` | double | Lidar alignment tolerances, speeds and limits |
| `retract_arm_before_navigation` | bool | Arm retract at the start of every step |
| `lift_retract_position`, `lift_up_position`, `lift_min_height` | double | Travel height, lift-up target and its success threshold (m) |
| `wrist_*_stow_position` | double | Wrist stow orientation (rad) |
| `lift_before_policy` | bool | Lift to the top before each policy |
| `policy_start_timeout`, `policy_timeout`, `command_timeout` | double | Waits for `/policy/start`, for `STOPPED`, and for a command step without its own timeout (s) |

| Topic | Type | Notes |
|---|---|---|
| `mission/state` (pub) | `std_msgs/String` | Mission progress as JSON: state, step, and the status of every phase of every step. Transient local. |
| `/wheel_odom` (sub) | `nav_msgs/Odometry` | Stationary check and alignment moves |
| `/joint_states` (sub) | `sensor_msgs/JointState` | Arm, lift and wrist positions |
| `/scan_filtered` (sub) | `sensor_msgs/LaserScan` | Lidar alignment |
| `/cmd_vel` (pub) | `geometry_msgs/Twist` | Alignment moves only, never while Nav2 has a goal |

It uses `/follow_joint_trajectory` (action), Nav2's `navigate_to_pose`, and the bridge's `/policy/start`, `/policy/stop` and `/policy/status`.

### policy_bridge

Node `/stretch4_policy_bridge`, started with the robot parts by `navigation.launch.py`. Machine settings come from `config/policy_runner.yaml`; the per-policy parameters (`task`, `root`, `pose_file`, `ckpt`, `run_args`, `max_runtime_sec`, `runner`, `dataset`, `policy_type`) are set by the mission before each policy step.

| Parameter | Type | Description |
|---|---|---|
| `robot_repo`, `jetson_repo` | string | `stretch4_policy_recorder` checkouts with their `.venv` (`recorder`) |
| `lerobot_robot_repo`, `lerobot_jetson_repo` | string | `lerobot_stretch4` checkouts with their `.venv` (`lerobot`) |
| `jetson_host` | string | ssh target of the Jetson, e.g. `jetson1@192.168.1.101` |
| `ssh_options` | string[] | Options for every ssh call (key login, connect timeout) |
| `goto_extra_args` | string | Extra `stretch4_goto_pose` arguments |
| `goto_timeout` | double | Start-pose move timeout (s) |
| `lerobot_server_port`, `lerobot_server_timeout` | int, double | `stretch4_policy_server` port, and how long a server started by the bridge may take to listen (s) |
| `start_delay` | double | Wait after starting the robot-side processes (s) |
| `stop_timeout` | double | Wait per stop stage, Ctrl-C then SIGTERM (s) |
| `log_dir` | string | Log folder (`~/.ros/stretch4_mission`) |

| Service | Type | Notes |
|---|---|---|
| `/policy/start` | `std_srvs/Trigger` | Move to the start pose, then start the policy. Blocks during the start-pose move. |
| `/policy/stop` | `std_srvs/Trigger` | Stop the running policy in stop order |
| `/policy/status` | `std_srvs/Trigger` | `RUNNING`, `STOP_REQUESTED` or `STOPPED`, with the log prefix. `STOPPED` describes the processes, not task success. |

### mission_ui

The web UI (HTTP on `127.0.0.1:8090`). It starts `navigation.launch.py` once per robot part and `mission.launch.py` per mission.

| Parameter | Type | Description |
|---|---|---|
| `host`, `port` | string, int | HTTP address (default `127.0.0.1`, 8090) |
| `missions_dir` | string | Mission files and `scans/` |
| `maps_dir`, `default_map` | string | Map folder and the preselected map |
| `policies_file` | string | Policy library |
| `launch_args` | string[] | Extra arguments for every launch (the script sets `use_rviz:=false`) |
| `localize_spins` | int | Turns allowed for global localization |
| `localized_sigma_xy`, `localized_sigma_yaw` | double | AMCL spread that counts as localized (m, rad) |
| `trajectory_action` | string | Driver action checked for readiness |
| `planner_python` | string | Python of the planner venv for text requests (default `~/.venvs/stretch4_planner/bin/python`) |
| `planner_model` | string | Claude model for text requests (default `claude-opus-5-5`) |

### Launch files

| Launch file | Arguments | Purpose |
|---|---|---|
| `navigation.launch.py` | `driver`, `lidar`, `nav2`, `bridge` (each `true`/`false`), `map`, `use_rviz`, `tool_preset`, `action_timeout`, `policy_runner` | Any combination of the robot parts; the UI starts each part with only its switch on |
| `mission.launch.py` | `mission_file` or `goal`, `navigation`, `map`, `mission_config`, `policies_file`, `use_rviz`, `launch_driver`, `launch_bridge`, `policy_runner` | One mission, plus (`navigation:=true`) the navigation session for its map |

## Command line (no UI)

```bash
ros2 launch stretch4_mission mission.launch.py mission_file:=$HOME/ament_ws/src/stretch4_mission/missions/<name>.yaml
```

This starts navigation on the mission's map plus the bridge, then the mission. Localize with RViz **2D Pose Estimate**, or set `set_initial_pose` in `config/mission.yaml`. Use `navigation:=false` when a navigation session (for example the UI's) is already running.

## Safety

### Controller arbitration

- Nav2 controls the base only through `cmd_vel` → `stretch_driver` (priority 0). The arm and lift moves use the driver's trajectory action, only while no Nav2 goal is active.
- The start-pose move and the policy processes run only inside a step's policy phase, after Nav2 has finished and the base is stopped. With `--base` (`recorder`) or `--robot.use_base=true` (`lerobot`) the policy may drive the base, so the robot is not at the goal afterwards.
- A command step runs only after the arm retract, with no Nav2 goal and no policy running.
- The mission starts the next step only after the current one has ended (`STOPPED` for a policy, exit for a command). It never runs two of these at once.

### Failure behaviour

Any step that fails → `FAILED`, and nothing after it runs:

- Nav2 not active, not localized, goal rejected/aborted/timed out, base not stopped, final pose outside tolerance
- arm/lift trajectory rejected or not settled at the required position
- before the first step: `max_runtime_sec: auto` with no recorded episodes, or a limit not below `policy_timeout`
- bridge parameters rejected, policy already running, `/policy/start` refused (start-pose move failed or timed out; `recorder`: stream/exec exited at startup, ssh to the Jetson failed or `stretch4_policy_run` exited at once; `lerobot`: dataset or model missing, the server did not listen, `robot_client` exited at once — each says which log to read), policy timeout (→ `/policy/stop`)
- command step: non-zero exit code, or still running after its timeout (→ Ctrl-C, then kill)

`/policy/start` blocks while the start-pose move runs (up to `goto_timeout`, 120 s), plus up to `lerobot_server_timeout` (60 s) for a LeRobot server; `policy_start_timeout` (200 s) covers both.

**NOTE**: ROS services are not a safety system. Keep the physical stop within reach.

## Testing

The pytest suite in */test* needs no robot. Run it with colcon:

```bash
$ cd ~/ament_ws
$ colcon test --packages-select stretch4_mission
$ colcon test-result --verbose
```

Or directly, from the package source:

```bash
$ cd ~/ament_ws/src/stretch4_mission
$ python3 -m pytest -q
```

## License

Apache 2.0. Please see the [LICENSE](./LICENSE) file.
