# Installation

One-time setup of a robot for *stretch4_mission*, then the read-only check before each session.

## One-time setup on the robot

1. **Policy runners.** Install the tools of the runners you use, each with its `.venv`, on the robot and on the Jetson: `stretch4_policy_recorder` in `~/repos/stretch4_policy_recorder` for the `recorder` runner, `lerobot_stretch4` in `~/repos/lerobot_stretch4` for the `lerobot` runner. The robot must log in to the Jetson with an ssh key (no password): `ssh -o BatchMode=yes jetson1@192.168.1.101 true` on the robot must succeed. Settings: `config/policy_runner.yaml`.
2. **Each policy needs** a folder on the robot with `start_pose.json` and, optionally, more pose JSON files beside it (each mission step selects one), and its model on the Jetson. For the `recorder` runner the folder is the dataset (`~/data/<name>`); `stretch4_goto_pose --root <dataset> --pose-file <json> --show` prints a pose. For the `lerobot` runner it is `~/repos/lerobot_stretch4/policies/<name>`, and the robot also needs the LeRobot dataset.
3. **Add each policy** to `config/policies.yaml` (templates at the bottom of the file). `scripts/run_ui_robot.sh` does this for you: before the UI starts it runs `sync_policies` (see [Adding policies](policy-steps.md#adding-policies)).
4. **Build** as in the [Quickstart](../README.md#quickstart). The install is a copy, so rebuild after code changes. Policies, missions and `config/mission.yaml` are read from `~/ament_ws/src/stretch4_mission/`, so editing them needs no rebuild (`mission.yaml` only when started through `ros2 launch` without `mission_config:=`).
5. **Text requests (optional)**, for [Text requests](operator.md#text-requests). On the robot, with internet:

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
6. **Admin password**, for the UI on the local network ([Operators and admins](web-ui.md#operators-and-admins)): `ros2 run stretch4_mission set_admin_password` on the robot. Without it, only the SSH tunnel gives admin access.
7. **On another robot**, nothing changes for the robot itself: `scripts/run_ui_robot.sh` reads its ID from `/etc/hello-robot/hello-robot.conf` (`HELLO_FLEET_ID=stretch-se4-…`), which every Stretch has, so the driver and lidars load that robot's calibration. Do check, on that robot:
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
