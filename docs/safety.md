# Safety

## Controller arbitration

- Nav2 controls the base only through `cmd_vel` → `stretch_driver` (priority 0). The arm and lift moves use the driver's trajectory action, only while no Nav2 goal is active.
- The start-pose move and the policy processes run only inside a step's policy phase, after Nav2 has finished and the base is stopped. With `--base` (`recorder`) or `--robot.use_base=true` (`lerobot`) the policy may drive the base, so the robot is not at the goal afterwards.
- A command step runs only after the arm retract, with no Nav2 goal and no policy running.
- The mission starts the next step only after the current one has ended (`STOPPED` for a policy, exit for a command). It never runs two of these at once.

## Failure behaviour

Any step that fails → `FAILED`, and nothing after it runs:

- Nav2 not active, not localized, goal rejected/aborted/timed out, base not stopped, final pose outside tolerance
- arm/lift trajectory rejected or not settled at the required position
- before the first step: `max_runtime_sec: auto` with no recorded episodes, or a limit not below `policy_timeout`
- bridge parameters rejected, policy already running, `/policy/start` refused (start-pose move failed or timed out; `recorder`: stream/exec exited at startup, ssh to the Jetson failed or `stretch4_policy_run` exited at once; `lerobot`: dataset or model missing, the server did not listen, `robot_client` exited at once — each says which log to read), policy timeout (→ `/policy/stop`)
- command step: non-zero exit code, or still running after its timeout (→ Ctrl-C, then kill)

`/policy/start` blocks while the start-pose move runs (up to `goto_timeout`, 120 s), plus up to `lerobot_server_timeout` (60 s) for a LeRobot server; `policy_start_timeout` (200 s) covers both.

**NOTE**: ROS services are not a safety system. Keep the physical stop within reach.
