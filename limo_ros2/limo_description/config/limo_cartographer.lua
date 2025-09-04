-- Minimal Cartographer configuration for the Limo robot.

-- Define a 2D trajectory builder and map builder directly so this file is
-- self-contained and does not rely on Cartographer's example config files.

MAP_BUILDER = {
  use_trajectory_builder_2d = true,
  use_trajectory_builder_3d = false,
  num_background_threads = 4,
}

TRAJECTORY_BUILDER_2D = {
  min_range = 0.3,
  max_range = 8.0,
  missing_data_ray_length = 5.,
  use_imu_data = false,
  voxel_filter_size = 0.025,
  submaps = {
    resolution = 0.05,
    num_range_data = 90,
  },
}

TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
}

POSE_GRAPH = {
  optimize_every_n_nodes = 90,
}

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

return options
