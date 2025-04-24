#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Locate package share directory
    bringup_dir = get_package_share_directory('dynabot')

    # Paths to existing launch files
    navigation_launch_file = os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
    online_launch_file = os.path.join(bringup_dir, 'launch', 'online_async_launch.py')

    # Declare common launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file for navigation nodes')
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file', default_value=os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file for the SLAM node')
    use_composition_arg = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Use composed bringup if True')
    container_name_arg = DeclareLaunchArgument(
        'container_name', default_value='nav2_container', description='Container name for composed nodes')
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='False', description='Respawn crashed nodes when composition is disabled')
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level for all nodes')

    # Include navigation launch
    nav_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('params_file'),
            'use_composition': LaunchConfiguration('use_composition'),
            'container_name': LaunchConfiguration('container_name'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )

    # Include online async SLAM launch
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(online_launch_file),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items()
    )

    # Assemble and return launch description
    ld = LaunchDescription()
    ld.add_action(namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(autostart_arg)
    ld.add_action(params_file_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(container_name_arg)
    ld.add_action(use_respawn_arg)
    ld.add_action(log_level_arg)

    ld.add_action(nav_include)
    ld.add_action(slam_include)

    return ld
