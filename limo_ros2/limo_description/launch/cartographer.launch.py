import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  pkg_share = get_package_share_directory('limo_description')
  config_dir = os.path.join(pkg_share, 'config')
  config_basename = 'limo_cartographer.lua'

  return LaunchDescription([
    Node(
      package='cartographer_ros',
      executable='cartographer_node',
      name='cartographer_node',
      output='screen',
      parameters=[{'use_sim_time': True}],
      arguments=[
        '-configuration_directory', config_dir,
        '-configuration_basename', config_basename
      ],
    ),
    Node(
      package='cartographer_ros',
      executable='occupancy_grid_node',
      name='occupancy_grid_node',
      output='screen',
      parameters=[{'use_sim_time': True}],
      arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
    ),
  ])
