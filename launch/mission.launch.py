"""One mission: the mission controller, plus (navigation:=true) the navigation session for its map.

The UI starts navigation.launch.py itself and runs missions with navigation:=false.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    mission_file = os.path.expanduser(LaunchConfiguration('mission_file').perform(context))
    if not mission_file:
        raise RuntimeError('mission_file:=<missions/name.yaml> is required')
    actions = []

    if LaunchConfiguration('navigation').perform(context) == 'true':
        map_path = LaunchConfiguration('map').perform(context)
        if not map_path:
            with open(mission_file) as f:
                map_path = (yaml.safe_load(f) or {}).get('map', '')
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('stretch4_mission'), 'launch', 'navigation.launch.py']),
            launch_arguments={
                'map': map_path,
                'use_rviz': LaunchConfiguration('use_rviz'),
                'driver': LaunchConfiguration('launch_driver'),
                'lidar': 'true',
                'nav2': 'true',
                'bridge': LaunchConfiguration('launch_bridge'),
                'policy_runner': LaunchConfiguration('policy_runner'),
            }.items(),
        ))

    overrides = {'mission_file': mission_file}
    policies = LaunchConfiguration('policies_file').perform(context)
    if policies:
        overrides['policies_file'] = policies
    actions.append(Node(
        package='stretch4_mission',
        executable='mission_controller',
        name='stretch4_mission_controller',
        parameters=[os.path.expanduser(LaunchConfiguration('mission_config').perform(context)), overrides],
        output='screen',
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_file', default_value='', description='missions/<name>.yaml'),
        DeclareLaunchArgument('mission_config',
                              default_value=PathJoinSubstitution([FindPackageShare('stretch4_mission'), 'config', 'mission.yaml']),
                              description='Tuning parameters (tolerances, joints, timeouts)'),
        DeclareLaunchArgument('policies_file', default_value='',
                              description='Policy library; empty = policies_file from mission_config'),
        DeclareLaunchArgument('navigation', default_value='true', choices=['true', 'false'],
                              description='false when a navigation session is already running (UI)'),
        DeclareLaunchArgument('map', default_value='', description='Overrides the mission file map'),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('launch_driver', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('launch_bridge', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('policy_runner', default_value='~/ament_ws/src/stretch4_mission/config/policy_runner.yaml'),
        OpaqueFunction(function=launch_setup),
    ])
