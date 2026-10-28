# Reference

The nodes, their parameters, topics and services, the launch files, and running a mission without the UI.

## mission_controller

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

## policy_bridge

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
| `snapshot_cameras` | string[] | `recorder`: stream cameras pictured when a policy stops, for the Operator view (`[]` = none) |
| `stop_timeout` | double | Wait per stop stage, Ctrl-C then SIGTERM (s) |
| `log_dir` | string | Log folder (`~/.ros/stretch4_mission`) |

| Service | Type | Notes |
|---|---|---|
| `/policy/start` | `std_srvs/Trigger` | Move to the start pose, then start the policy. Blocks during the start-pose move. |
| `/policy/stop` | `std_srvs/Trigger` | Stop the running policy in stop order |
| `/policy/status` | `std_srvs/Trigger` | `RUNNING`, `STOP_REQUESTED` or `STOPPED`, with the log prefix. `STOPPED` describes the processes, not task success. |

## mission_ui

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

## Launch files

| Launch file | Arguments | Purpose |
|---|---|---|
| `navigation.launch.py` | `driver`, `lidar`, `nav2`, `bridge` (each `true`/`false`), `map`, `use_rviz`, `tool_preset`, `action_timeout`, `policy_runner` | Any combination of the robot parts; the UI starts each part with only its switch on |
| `mission.launch.py` | `mission_file` or `goal`, `navigation`, `map`, `mission_config`, `policies_file`, `use_rviz`, `launch_driver`, `launch_bridge`, `policy_runner` | One mission, plus (`navigation:=true`) the navigation session for its map |

## Command line (no UI)

```bash
ros2 launch stretch4_mission mission.launch.py mission_file:=$HOME/ament_ws/src/stretch4_mission/missions/<name>.yaml
```

This starts navigation on the mission's map plus the bridge, then the mission. Localize with RViz **2D Pose Estimate**, or set `set_initial_pose` in `config/mission.yaml`. Use `navigation:=false` when a navigation session (for example the UI's) is already running.
