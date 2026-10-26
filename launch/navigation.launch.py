"""Start any combination of the robot driver, lidars, Nav2, and policy bridge.

In simulation, the existing MuJoCo driver provides the driver and lidar nodes.
"""

import os

from hello_helpers.multi_yaml import MultiYaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    def enabled(name):
        return LaunchConfiguration(name).perform(context) == 'true'
    sim = enabled('sim')
    core, nav2 = FindPackageShare('stretch_core'), FindPackageShare('stretch_nav2')
    tool = LaunchConfiguration('tool_preset')
    actions = []

    if enabled('driver') and not sim:
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([core, 'launch', 'stretch_driver.launch.py']),
            launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation',
                              'action_timeout': LaunchConfiguration('action_timeout')}.items()))
    if enabled('lidar') and not sim:
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([core, 'launch', 'dual_hesai.launch.py']),
            launch_arguments={'filter_type': 'sor_ransac', 'tool_preset': tool, 'use_rviz': 'false'}.items()))
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([core, 'launch', 'robot_footprint.launch.py']),
            launch_arguments={'tool_preset': tool}.items()))
    if enabled('nav2'):
        map_path = os.path.expanduser(LaunchConfiguration('map').perform(context))
        if not map_path:
            raise RuntimeError('nav2:=true needs map:=<map.yaml>')
        params = [PathJoinSubstitution([nav2, 'config', f]) for f in
                  ('original_nav2_params.yaml', 'nav2_params_core.yaml', 'nav2_params_mppi.yaml', 'mppi_params.yaml')]
        if sim:
            # Wall time, not use_sim_time: the sim driver stamps odom/TF/scan with its wall-clock node time.
            params.append(PathJoinSubstitution([FindPackageShare('stretch4_mission'), 'config', 'sim_nav2_overrides.yaml']))
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([nav2, 'launch', 'include', 'nav_core.launch.py']),
            launch_arguments={'map': map_path, 'params_file': MultiYaml(params),
                              'use_rviz': LaunchConfiguration('use_rviz'),
                              'rviz_config': PathJoinSubstitution([nav2, 'rviz', 'navigation.rviz']),
                              'use_composition': 'True'}.items()))
    if enabled('bridge'):
        if sim:
            actions.append(Node(package='stretch4_mission', executable='sim_policy_bridge', output='screen'))
        else:
            actions.append(Node(package='stretch4_mission', executable='policy_bridge', name='stretch4_policy_bridge',
                                parameters=[os.path.expanduser(LaunchConfiguration('policy_runner').perform(context))],
                                output='screen'))
    if not actions:
        raise RuntimeError('nothing to start: set at least one of driver, lidar, nav2, bridge to true')
    # GroupAction scopes each include's launch_arguments (same reason as navigation_mppi.launch.py's scoped()).
    return [GroupAction([a]) for a in actions]


def generate_launch_description():
    def flag(name, description):
        return DeclareLaunchArgument(name, default_value='true', choices=['true', 'false'], description=description)
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='', description='Map yaml (map_saver_cli output), for nav2'),
        flag('driver', 'stretch_driver in navigation mode (robot only)'),
        flag('lidar', 'dual Hesai lidars + /scan_filtered + robot footprint (robot only)'),
        flag('nav2', 'Nav2: map server, AMCL, planners, controller'),
        flag('bridge', 'policy bridge (/policy/start, /stop, /status)'),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('tool_preset', default_value='auto',
                              description='Mounted tool preset for the lidar self-filter: auto, sg4, pg4, tablet, or nil'),
        DeclareLaunchArgument('action_timeout', default_value='30.0',
                              description='Default timeout (s) of the driver joint trajectory action'),
        DeclareLaunchArgument('policy_runner', default_value='~/tec_workspace/stretch4_mission/config/policy_runner.yaml',
                              description='policy_bridge settings (robot repo, Jetson host, ssh)'),
        DeclareLaunchArgument('sim', default_value='false', choices=['true', 'false'],
                              description='true: MuJoCo sim (sim driver/lidar, sim Nav2 overrides, sim_policy_bridge)'),
        OpaqueFunction(function=launch_setup),
    ])
