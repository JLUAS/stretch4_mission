# Navigate steps

A navigate step drives the robot to a goal on the map and stops there, aligned to the spot when the step has a lidar reference. A [policy step](policy-steps.md) does the same before its policy. In the editor, drive the robot to the spot and click **Use robot pose as goal**; give the step a label ("kitchen") so [text requests](operator.md#text-requests) can use it as a place.

## Navigation and alignment

- **Navigation:** before each goal, the mission tightens Nav2's goal checker at runtime to 0.10 m / 0.08 rad (`stretch_nav2` accepts 0.25 m / 0.34 rad). After success it waits for the base to stop, then checks 0.25 m / 0.15 rad itself.
- **Align (lidar):** AMCL alone is good to about 5-10 cm and can jump after the robot stops, too coarse for a policy trained from a fixed spot. For steps with a lidar reference, the robot waits until it is still, checks AMCL is within 0.40 m / 0.50 rad, then matches the live scan to the reference (ICP) and corrects with slow omni-base moves on odometry, repeating until within 1 cm / 0.01 rad (`align_*` in `config/mission.yaml`). It fails without starting the policy if the scans match poorly (surroundings changed, < 60 % of points), the correction is too large, or something comes within 0.26 m of the base center. Steps without a reference use the AMCL check (0.25 m / 0.15 rad) instead. Measured on the robot: two captures of the same spot agree within 0.7 mm / 0.01 deg (96 % match).

**Tip**: Keep the area around each station as it was when its lidar reference was captured, and capture it again if furniture moves.

## Go to pose

**Go to pose** (under the map) drives the robot to any spot without a mission: press on the map where it should go and drag toward where it should face (a teal arrow), then click **Go to pose**. The robot stows the arm with stretch_nav_webapp's stow (the robot's own stow pose; the robot must be homed), then Nav2 drives there through the webapp's NavigateToPose client. It shows in the state machine as a one-step navigate mission (alignment skipped), and **Stop mission** stops it. It needs the same as a mission: Nav2 on the map shown, every part ready, the robot localized. From a terminal: `ros2 launch stretch4_mission mission.launch.py navigation:=false goal:=x,y,yaw`.
