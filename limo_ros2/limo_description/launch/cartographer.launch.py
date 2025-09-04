# save as: cartographer_gazebo_embedded.launch.py
import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LUA_TEMPLATE = r'''
include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "{tracking_frame}",
  published_frame = "{published_frame}",
  odom_frame = "{odom_frame}",

  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,

  use_odometry = {use_odometry},
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.5,   -- Gazebo에서 TF 초기 지연 대비
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 0.03,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D LIDAR 기본값 (Gazebo)
TRAJECTORY_BUILDER_2D.min_z = -0.2
TRAJECTORY_BUILDER_2D.max_z =  1.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- 스캔 매칭 튜닝(시뮬 안정화)
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.optimize_every_n_nodes = 30

return options
'''

def _setup(context, *args, **kwargs):
    # 런치 인자 평가
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    scan_topic   = LaunchConfiguration('scan_topic').perform(context)
    use_odom     = LaunchConfiguration('use_odometry').perform(context)
    inject_dummy = LaunchConfiguration('inject_dummy_odom_tf').perform(context)
    tracking     = LaunchConfiguration('tracking_frame').perform(context)
    published    = LaunchConfiguration('published_frame').perform(context)
    odom         = LaunchConfiguration('odom_frame').perform(context)
    res          = LaunchConfiguration('map_resolution').perform(context)
    pub_period   = LaunchConfiguration('publish_period_sec').perform(context)

    # 임시 Lua 파일 생성 (인자값 반영)
    tmp_dir = tempfile.mkdtemp(prefix="carto_gazebo_")
    cfg_name = "embedded_cartographer.lua"
    cfg_path = os.path.join(tmp_dir, cfg_name)
    lua_text = LUA_TEMPLATE.format(
        tracking_frame=tracking,
        published_frame=published,
        odom_frame=odom,
        use_odometry=use_odom
    )
    with open(cfg_path, "w") as f:
        f.write(lua_text)

    nodes = []

    # Cartographer
    nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            arguments=[
                '-configuration_directory', tmp_dir,
                '-configuration_basename', cfg_name,
            ],
            remappings=[('scan', scan_topic)],
        )
    )

    # Occupancy Grid
    nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time == 'true'}],
            arguments=[
                '-resolution', res,
                '-publish_period_sec', pub_period
            ],
        )
    )

    # (옵션) 더미 odom→base_link TF (항등) 주입: 빠른 시뮬 확인용
    if inject_dummy == 'true':
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='dummy_odom_base_link_tf',
                output='screen',
                arguments=['0','0','0','0','0','0','1', odom, tracking],
            )
        )

    return nodes

def generate_launch_description():
    declares = [
        # Gazebo는 /clock을 내보내므로 기본 true
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Gazebo LIDAR 토픽(예: /scan, /lidar/scan 등)
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        # 오도메트리 없으면 false 권장
        DeclareLaunchArgument('use_odometry', default_value='false', choices=['true','false']),
        # 급할 때 테스트용: odom→base_link 항등 TF
        DeclareLaunchArgument('inject_dummy_odom_tf', default_value='false', choices=['true','false']),

        # 프레임 이름 (로봇 URDF/TREE에 맞춰 조정)
        DeclareLaunchArgument('tracking_frame', default_value='base_link'),
        DeclareLaunchArgument('published_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),

        # 맵 해상도/발행주기
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
    ]
    return LaunchDescription(declares + [OpaqueFunction(function=_setup)])
