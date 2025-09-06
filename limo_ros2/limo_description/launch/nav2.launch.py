#!/usr/bin/env python3
"""Launch Navigation2 for the Limo robot."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('limo_description')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the ROS2 parameters file to use')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch', 'bringup_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params_file}.items())

    return LaunchDescription([
        declare_sim_time,
        declare_params_file,
        bringup_cmd,
    ])