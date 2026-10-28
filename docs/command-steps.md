# Command steps

A command step runs one line with `bash -c` on the robot, as the user that started the UI, in the home folder, with the UI's environment (ROS sourced, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`), so a Python script can use `rclpy` and `stretch_driver`. It runs where the robot is, after the arm retract; put a **Navigate only** step before it to move first. Exit code 0 = the step passed; any other code fails the mission. After `timeout_sec` (or `command_timeout` in `config/mission.yaml`, default 300 s; at most 1000 s) the mission sends Ctrl-C to the command's process group, waits 5 s, then kills what is left. **Stop mission** does the same. Output: `~/.ros/stretch4_mission/command_<time>.log`.

**NOTE**: `stretch_driver` stays in navigation mode and nothing else commands the robot while the command runs; the script is responsible for what it moves.
