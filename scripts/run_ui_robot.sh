#!/usr/bin/env bash
# Start the mission UI on the robot with the same environment as ~/.bashrc (works from a plain ssh command too).
# Open it from your computer through a tunnel:  ssh -L 8090:localhost:8090 hello-robot@<robot-ip>  ->  http://localhost:8090
# Extra arguments are passed to the node, e.g.:  -p host:=0.0.0.0   (serve the whole network; the page has no login)
# In the background: setsid -f <this script> > ~/.ros/stretch4_mission/ui.log 2>&1   (not `&`: it makes Ctrl-C ignored)
set -e
export HELLO_FLEET_PATH=/home/hello-robot/stretch_user
# This robot's own ID (each Stretch has it there); a fixed ID would load another robot's calibration.
. /etc/hello-robot/hello-robot.conf
export HELLO_FLEET_ID
export PATH="/home/hello-robot/.local/bin:$PATH"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
set +u
source /opt/ros/jazzy/setup.bash
source /home/hello-robot/ament_ws/install/setup.bash
# New policies go into config/policies.yaml: every ~/data/<name> with a <name>.ckpt or <name>_*.ckpt in the Jetson's
# ~/policies, and every ~/repos/lerobot_stretch4/policies/<name> with ~/policies/<name>/pretrained_model there.
ros2 run stretch4_mission sync_policies || echo "sync_policies failed: the UI shows only the policies already in config/policies.yaml"
exec ros2 run stretch4_mission mission_ui --ros-args \
  -p maps_dir:=$HELLO_FLEET_PATH/maps \
  -p default_map:=$HELLO_FLEET_PATH/maps/test.yaml \
  -p "launch_args:=['use_rviz:=false']" "$@"
