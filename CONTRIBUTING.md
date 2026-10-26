# Contributing

This is an independent ROS 2 project for Stretch 4. Contributions should remain focused, preserve the existing
architecture, and include documentation or tests when behavior changes.

## Development setup

Use Ubuntu 24.04 with ROS 2 Jazzy and a workspace that already contains the Stretch 4 ROS 2 dependencies.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ament_ws/install/setup.bash
cd ~/ament_ws
colcon build --packages-select stretch4_mission
source install/setup.bash
```

## Validation

Run the package tests before opening a pull request:

```bash
cd ~/ament_ws/src/stretch4_mission
python3 -m pytest -q
```

For changes to launch files, packaging, or ROS dependencies, also rebuild the package with `colcon`.

## Pull requests

- Keep each pull request limited to one coherent change.
- Explain what changed, how it was validated, and what was not tested.
- Update the README when setup, configuration, or visible behavior changes.
- Do not commit credentials, checkpoints, policy datasets, maps, recorded missions, or robot logs.
- Do not include third-party logos or imply endorsement by Hello Robot.

## Robot safety

Prefer unit tests before hardware testing. State explicitly when a change was only statically tested.
Hardware tests must keep the physical stop accessible and must not bypass joint limits, collision
checks, watchdogs, or other safety mechanisms.

