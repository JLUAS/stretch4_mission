# stretch4_mission

Multi-policy missions for Stretch 4 (ROS 2 Jazzy, `rmw_zenoh_cpp`), with a web UI.

![Stretch 4 Missions web UI](ui.png)

A **mission** is a map plus ordered **steps**. Each step is one trained policy (run with
`stretch4_policy_recorder`) with a navigation goal; the start pose comes from the policy's dataset. Example "clean the table": open cabinet → pick sponge → clean table. For each step the robot does:

**lift to 0.5 m → retract arm → stow wrist only → check pose → Nav2 to the goal if needed → verify arrival + stationary → lift to max →
`/policy/start` (bridge: `stretch4_goto_pose --root <dataset>`, then the policy) → wait for `STOPPED`**

## Repository layout

```text
.github/                 Pull-request guidance
config/                  Mission and policy-runner configuration
launch/                  ROS 2 launch files
missions/                Local robot missions (ignored by Git)
scripts/                 Operator entry points
stretch4_mission/        Python package and web UI
test/                    Pytest suite
```

| Piece | Existing component |
|---|---|
| Navigation, AMCL, obstacle avoidance | `stretch_nav2` `navigation_mppi.launch.py` (also starts `stretch_driver` in `navigation` mode, the dual Hesai lidars and footprint) |
| Global localization | AMCL `reinitialize_global_localization` + `request_nomotion_update`, Nav2 `spin` action |
| Goal | Nav2 `NavigateToPose` via `nav2_simple_commander.BasicNavigator` |
| Pose / stationary | TF2 `map` → `base_link` / `/wheel_odom` |
| Arm retract, lift up | `stretch_driver`'s `/stretch_controller/follow_joint_trajectory`, one joint at a time |
| Policies | `policy_bridge` (this package): `stretch4_goto_pose`, `stretch4_policy_stream` and `stretch4_policy_exec` on the robot, `stretch4_policy_run` on the Jetson over ssh, behind `/policy/start`, `/stop`, `/status`; the mission sets its parameters per step |

## Files

| File | What |
|---|---|
| `config/policies.yaml` | Policy library. Each entry: `task`, `root` (robot: dataset folder containing selectable pose JSON files), `ckpt` (Jetson: checkpoint), `run_args` (e.g. `--fp16 --base`), `max_runtime_sec` (seconds, or `auto`; see [Policy run time](#policy-run-time)), optional `runtime_margin_sec`. Top-level `runtime_margin_sec`: the margin `auto` adds unless a policy sets its own. **Add your policies here.** |
| `missions/<name>.yaml` | Missions (`name`, `map`, `steps: [{policy, start_pose, goal: {x, y, yaw}, scan, max_runtime_sec (optional)}]`). Written by the UI. `start_pose` is a JSON file in the policy root; `scan` is the step's lidar reference (`missions/scans/`). |
| `config/mission.yaml` | Tuning for all missions: tolerances, joints, lift height, timeouts. |
| `config/policy_runner.yaml` | Policy runner machine settings: robot repo, Jetson host (`jetson1@192.168.1.101`), Jetson repo, ssh options, timeouts. |

## One-time setup on the robot

1. **Policy runner.** `stretch4_policy_recorder` must be installed with its `.venv` on the robot and on the Jetson,
   and the robot must log in to the Jetson with an ssh key (no password): `ssh -o BatchMode=yes jetson1@192.168.1.101
   true` on the robot must succeed (checked 2026-10-26). Settings: `config/policy_runner.yaml`.
2. **Each policy needs** its dataset on the robot with `start_pose.json` and optionally more pose JSON files beside
   it. Each mission step selects one; `stretch4_goto_pose --root <dataset> --pose-file <json> --show` prints it.
   The policy also needs its checkpoint on the Jetson.
3. **Add each policy** to `config/policies.yaml` (copy the template at the bottom of the file).
4. **Build:**
   ```bash
   cd ~/ament_ws/src
   git clone git@github.com:JLUAS/stretch4_mission.git
   cd ~/ament_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select stretch4_mission
   source install/setup.bash
   ```
   The install is a copy, so rebuild after code changes. Policies, missions and `config/mission.yaml` are read from
   `~/ament_ws/src/stretch4_mission/`, so editing them needs no rebuild (`mission.yaml` only when started via
   `ros2 launch` without `mission_config:=`).

## Use the web UI

Before you start:
- Home the robot.
- Clear the area. The robot spins in place to localize, and lifts to the top before every policy.
- Keep a person with the physical stop next to the robot.

On the robot (it loads the same environment as `~/.bashrc`, so it also works from a plain `ssh` command):

```bash
~/ament_ws/src/stretch4_mission/scripts/run_ui_robot.sh
```

The UI listens only on the robot itself (`127.0.0.1:8090`). From your computer, open an SSH tunnel and then
http://localhost:8090:

```bash
ssh -N -L 8090:localhost:8090 hello-robot@<robot-ip>
```

The script preselects `$HELLO_FLEET_PATH/maps/test.yaml` and turns RViz off. Extra arguments go to the node; for
example `-p host:=0.0.0.0` serves the whole network at `http://<robot-ip>:8090`, but the page has no login.

1. **Robot:** choose the map, then **Start all**, or start each part with its own button: **Robot driver**,
   **Lidars**, **Nav2** (on the selected map) and **Policy bridge**. Together they are exactly what Hello Robot's
   `navigation_mppi.launch.py` starts (same included launch files and arguments), plus the bridge. Each row shows
   whether that part really works, not just whether its process exists: the driver's `/follow_joint_trajectory`
   action and `/joint_states`, fresh `/scan_filtered`, Nav2's `navigate_to_pose`, the bridge's `/policy/status`.
   Green = ready, yellow = started but not responding yet. A part already running outside the UI shows as
   "started outside the UI", and the UI will not start a second copy. Stop buttons send Ctrl-C; they are disabled
   while a mission runs. Each part has its own log (selector at the bottom).

   ![Step 1: choose a map and start the robot services](docs/ui/01-start-services.png)

2. **Localize (global):** AMCL spreads its guesses over the whole map. The robot rotates in place (Nav2 `spin`,
   collision-checked, up to 3 turns) until the estimate converges (σxy ≤ 0.25 m, σyaw ≤ 0.30 rad). Check that the
   blue robot on the map is where the real robot is; in rooms that look alike AMCL can converge to the wrong place.

   ![Step 2: global localization controls and status](docs/ui/02-localize.png)

3. **Create a mission:**
   1. Click **+ New mission**, give it a name and pick the map.
   2. For each policy, **+ Add policy step** and pick the policy. Its start pose is the policy's dataset (`root`).
      **Policy run time**, beside the start pose, shows how long the policy may run (see
      [Policy run time](#policy-run-time)), or why it is unknown.
   3. Drive the robot (gamepad) to **exactly** where that policy's episodes started, then click **Use robot pose as
      goal**. This copies the base pose (x, y, yaw from `map → base_link`) and saves a **lidar reference scan** of
      that spot (`missions/scans/`, median of 1 s of `/scan_filtered`). Goals appear as numbered arrows on the map;
      the step says whether it has a lidar reference.
   4. Reorder steps with ↑ ↓, then **Save mission**.

   ![Step 3: configure policies, start poses, and navigation goals](docs/ui/03-edit-mission.png)

4. **Start** the mission. The state machine shows every step and phase live: pending, active (blue), done (green),
   skipped (crossed out, e.g. navigation when already in place) and failed (red). **Stop mission** sends Ctrl-C; the
   mission cancels whatever it was doing. Navigation keeps running, so you can start another mission.

   ![Step 4: select a mission and follow its state machine](docs/ui/04-run-mission.png)

A mission only starts when Nav2 (started from the UI) runs on **its** map, the driver, lidars and bridge are
ready, and the robot is localized. The UI says which part is missing.

The UI has no login: anyone who can reach port 8090 can move the robot. Use it only on the lab network.

## Command line (no UI)

```bash
ros2 launch stretch4_mission mission.launch.py mission_file:=$HOME/ament_ws/src/stretch4_mission/missions/<name>.yaml
```

This starts navigation on the mission's map plus the bridge, then the mission. Localize with RViz
**2D Pose Estimate**, or set `set_initial_pose` in `config/mission.yaml`. Use `navigation:=false` when a navigation
session (for example the UI's) is already running. Other arguments: `mission_config`, `policies_file`, `map`,
`use_rviz`, `launch_driver`, `launch_bridge`, `policy_runner`.

## What happens between and inside steps

- **Arm and lift moves are judged on the measured joint**, not on the driver's action result. stretch_driver only
  finishes a trajectory after the joint has *started* moving, so a goal at the current position (arm already
  retracted, lift already at its top) would never finish. A joint already in place is skipped ("already there"),
  a move counts as done once the joint is in place and still (a goal the driver keeps tracking is cancelled), and
  a driver error fails the step at once.
- **Before every navigation, the lift moves to 0.5 m, the arm retracts, then the wrist moves to the SG4 stow
  orientation** (yaw 3.14, roll 0, pitch 0). The gripper is not commanded, so a held object stays held.
- **Navigation:** before each goal, the mission tightens Nav2's goal checker at runtime to 0.10 m / 0.08 rad
  (`stretch_nav2` accepts 0.25 m / 0.34 rad). After success it waits for the base to stop, then checks
  0.25 m / 0.15 rad itself.
- **Align (lidar)**: AMCL alone is good to ~5-10 cm and can jump after the robot stops, too coarse for a policy trained
  from a fixed spot. For steps with a lidar reference, the robot waits until it is still, checks AMCL is within 0.40 m
  / 0.50 rad, then matches the live scan to the reference (ICP) and corrects with slow omni-base moves on odometry,
  repeating until within 1 cm / 0.01 rad (`align_*` in `config/mission.yaml`). It fails without starting the policy
  if the scans match poorly (surroundings changed, < 60 % of points), the correction is too large, or something comes
  within 0.26 m of the base center. Keep the area around each station as it was when the reference was captured;
  re-capture it if furniture moves. Steps without a reference use the AMCL check (0.25 m / 0.15 rad) instead.
  Measured on the robot: two captures of the same spot agree within 0.7 mm / 0.01 deg (96 % match).
- **Lift to max** (`lift_up_position` 1.2 m, clipped by the driver). Success = measured lift ≥ `lift_min_height`
  (1.16 m) and settled. **Set `lift_min_height` above every start pose's lift.** `stretch4_goto_pose` may interpolate all
  joints without collision checking, so it should come down from the top.
- **Policy:** the mission sets the step's parameters on `/stretch4_policy_bridge` (all of them, every step, only while
  no policy runs) and calls `/policy/start`. The bridge runs `stretch4_goto_pose --root <root> --yes` (blocking), starts
  `stretch4_policy_stream` and `stretch4_policy_exec` on the robot, then `stretch4_policy_run --ckpt <ckpt> <run_args>
  --yes --duration <max_runtime_sec>` on the Jetson over ssh (`--yes`: there is no terminal to answer "Go?"). The
  mission polls `/policy/status` until `STOPPED`: the run ended (`--duration`), any of the three exited (the others are
  then stopped), or `max_runtime_sec` passed. Stop order: Jetson run first (no more actions; remote `pkill -INT`),
  then exec, then stream; Ctrl-C, then SIGTERM. stream/exec run only while a policy runs, so nothing else commands
  the robot during navigation. Logs: `~/.ros/stretch4_mission/policy_<time>_{goto,stream,exec,run}.log`. The next step starts after that. "Finished" means the
  policy **process** ended; it says nothing about whether the task succeeded. Check that physically.

## Policy run time

Each policy runs for at most `max_runtime_sec` (in `config/policies.yaml`). The bridge then stops it
(`stretch4_policy_run --duration`, plus its own watchdog), and the mission goes on with the next step.

- `max_runtime_sec: 45` is a fixed limit.
- `max_runtime_sec: auto` is the **longest episode recorded for that policy + `runtime_margin_sec`**:

  ```yaml
  runtime_margin_sec: 10.0        # top of config/policies.yaml; one value for every "auto" policy
  policies:
    grab_sponge:
      root: ~/data/grab_sponge
      max_runtime_sec: auto       # longest episode in ~/data/grab_sponge + 10 s
  ```

  A policy can set its own margin, which overrides the shared one:

  ```yaml
    clean_table:
      max_runtime_sec: auto
      runtime_margin_sec: 20.0    # longest clean_table episode + 20 s
  ```

  An episode's length is the time `t` of the last sample minus the first in
  `<root>/episode_*/records.jsonl` (written by `stretch4_policy_recorder`). Unreadable or half-written
  episodes are skipped. On 2026-10-26 the datasets gave: grab_sponge 26.0 s (→ 36.0 s), clean_table 68.7 s
  (→ 78.7 s), open_cabinet_v2 50.9 s.
- Without `max_runtime_sec` a policy gets 120 s.
- **Per mission step, in the UI:** the editor's **Policy run time (s)** field. Empty (the default) = the policy's
  value from `config/policies.yaml`, shown as the field's hint; a number = this step's own limit, saved in the
  mission file as the step's `max_runtime_sec`. It overrides the policy's value for that step only.

The mission works the limits out **when it starts, before the robot moves**, and logs one per step
(`step 1: grab_sponge: max runtime 36.0 s`). Recording more episodes changes the limit at the next mission
start; no rebuild needed. The UI shows the same values: on each mission card (per policy, plus the total
policy time, which does not include driving and alignment), per step in the state machine, and as **Policy run
time** in the editor (where it can be changed per step). `?` / unknown means `auto` found no recordings; the mission would refuse to start.

`policy_timeout` (`config/mission.yaml`, 300 s) is how long the mission waits for the bridge to report
`STOPPED`, so every limit must be below it; the mission checks this at start.

## Controller arbitration (why it is safe to run in sequence)

- Nav2 controls the base only through `cmd_vel` → `stretch_driver` (pushes at priority 0). The arm and lift steps
  use the driver's trajectory action, only while no Nav2 goal is active.
- `stretch4_goto_pose` and `stretch4_policy_exec` run only inside a step's policy phase, after Nav2 has finished and
  the base is stopped. With `--base` the policy may drive the base, so the robot is not at the goal afterwards.
- The mission starts a policy only after the navigation goal has finished and the base is stopped, and starts the
  next navigation only after the bridge reports `STOPPED`. It never runs two of these at once.

## Failure behaviour

Any step that fails → `FAILED`, and nothing after it runs:
- Nav2 not active, not localized, goal rejected/aborted/timeout, base not stopped, final pose outside tolerance
- arm/lift trajectory rejected or not settled at the required position
- before the first step: `max_runtime_sec: auto` with no recorded episodes in the policy's `root`, or a limit not
  below `policy_timeout`
- bridge parameters rejected, policy already running, `/policy/start` refused (goto failed or timed out, stream/exec
  exited at startup, ssh to the Jetson failed or `stretch4_policy_run` exited at once — each says which log to read),
  policy timeout (→ `/policy/stop`)

`/policy/start` blocks while `stretch4_goto_pose` runs (up to `goto_timeout`, 120 s); `policy_start_timeout` (150 s)
covers it.

ROS services are not a safety system. Keep the physical stop within reach.

## Test

```bash
cd ~/ament_ws && colcon test --packages-select stretch4_mission && colcon test-result --verbose
```
