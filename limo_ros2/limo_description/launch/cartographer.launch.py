#!/usr/bin/env python3
# save as: limo_description/launch/cartographer.launch.py
import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Lua 템플릿: Python .format() 과 충돌하지 않도록 중괄호는 {{ }}
LUA_TEMPLATE = r'''
include "map_builder.lua"
include "trajectory_builder.lua"

options = {{
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- frames
  map_frame = "map",
  tracking_frame = "{tracking_frame}",
  published_frame = "{published_frame}",
  odom_frame = "{odom_frame}",

  -- Gazebo가 /odometry(+ TF: odom->base_*) 를 제공한다고 가정
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,

  -- sensors
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  use_odometry = {use_odometry},
  use_nav_sat = false,
  use_landmarks = false,

  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 0.03,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0
}}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D Lidar 기본값(시뮬 안정화)
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- 매칭/최적화 튜닝
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.optimize_every_n_nodes = 30

return options
'''

def _setup(context, *args, **kwargs):
    # 런치 인자 평가
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    scan_topic   = LaunchConfiguration('scan_topic').perform(context)          # ex) /scan
    odom_topic   = LaunchConfiguration('odom_topic').perform(context)          # ex) /odometry (URDF 그대로)
    use_odom     = LaunchConfiguration('use_odometry').perform(context)        # 'true' or 'false'
    tracking     = LaunchConfiguration('tracking_frame').perform(context)      # base_link
    published    = LaunchConfiguration('published_frame').perform(context)     # base_link
    odom_frame   = LaunchConfiguration('odom_frame').perform(context)          # odom
    res          = LaunchConfiguration('map_resolution').perform(context)      # 0.05
    pub_period   = LaunchConfiguration('publish_period_sec').perform(context)  # 0.5

    inject_dummy = LaunchConfiguration('inject_dummy_odom_tf').perform(context)   # 'false'
    dummy_base   = LaunchConfiguration('dummy_base_frame').perform(context)       # base_footprint

    soft_reset   = LaunchConfiguration('soft_reset_on_start').perform(context)    # 'false'
    traj_id      = LaunchConfiguration('trajectory_id').perform(context)          # '0'

    # 임시 Lua 파일 생성
    tmp_dir = tempfile.mkdtemp(prefix="carto_cfg_")
    cfg_name = "embedded_cartographer.lua"
    cfg_path = os.path.join(tmp_dir, cfg_name)
    with open(cfg_path, "w") as f:
        f.write(LUA_TEMPLATE.format(
            tracking_frame=tracking,
            published_frame=published,
            odom_frame=odom_frame,
            use_odometry=use_odom
        ))

    nodes = []

    # Cartographer SLAM
    nodes.append(Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': (use_sim_time == 'true')}],
        arguments=['-configuration_directory', tmp_dir,
                   '-configuration_basename', cfg_name],
        # URDF는 /odometry 로 Odom 메시지를 내보내므로 여기서 리매핑
        remappings=[('scan', scan_topic), ('odom', odom_topic)],
    ))

    # Occupancy Grid 발행
    nodes.append(Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': (use_sim_time == 'true')}],
        arguments=['-resolution', res, '-publish_period_sec', pub_period],
    ))

    # (옵션) 더미 TF: odom -> base_footprint (실제 TF가 없을 때만 임시로 사용)
    if inject_dummy == 'true':
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='dummy_odom_to_base',
            output='screen',
            arguments=['0','0','0','0','0','0','1','odom', dummy_base],
        ))

    # (옵션) 소프트 리셋: 기존 trajectory 종료 후 즉시 새 trajectory 시작
    # - 완전 초기화(빈 맵)는 재실행이 가장 확실하지만,
    #   테스트 편의를 위해 서비스 호출을 넣어둠.
    if soft_reset == 'true':
        # 주의: cartographer_node가 서비스 준비될 시간을 조금 준다.
        # bash -c 로 sleep -> finish -> sleep -> start 순서 실행
        # StartTrajectory에는 방금 만든 cfg 경로를 넘겨준다.
        finish_cmd = (
            f"bash -lc \"sleep 1.0; "
            f"ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "
            f"{{trajectory_id: {traj_id}}}; "
            f"sleep 0.3; "
            f"ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory "
            f"{{configuration_directory: '{tmp_dir}', configuration_basename: '{cfg_name}', "
            f"use_initial_pose: false, initial_pose: {{position: {{x: 0.0, y: 0.0, z: 0.0}}, "
            f"orientation: {{w: 1.0}}}}, relative_to_trajectory_id: 0}}\""
        )
        nodes.append(ExecuteProcess(cmd=[finish_cmd], shell=True, output='screen'))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        # 공통
        DeclareLaunchArgument('use_sim_time',        default_value='true'),
        DeclareLaunchArgument('scan_topic',          default_value='/scan'),
        DeclareLaunchArgument('odom_topic',          default_value='/odometry'),  # ★ URDF에 맞춤
        DeclareLaunchArgument('use_odometry',        default_value='true', choices=['true','false']),

        # 프레임
        DeclareLaunchArgument('tracking_frame',      default_value='base_link'),
        DeclareLaunchArgument('published_frame',     default_value='base_link'),
        DeclareLaunchArgument('odom_frame',          default_value='odom'),

        # 맵 출력
        DeclareLaunchArgument('map_resolution',      default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec',  default_value='0.5'),

        # 디버그 옵션
        DeclareLaunchArgument('inject_dummy_odom_tf', default_value='false'),
        DeclareLaunchArgument('dummy_base_frame',     default_value='base_footprint'),

        # 소프트 리셋 옵션
        DeclareLaunchArgument('soft_reset_on_start',  default_value='false'),
        DeclareLaunchArgument('trajectory_id',        default_value='0'),

        OpaqueFunction(function=_setup)
    ])
