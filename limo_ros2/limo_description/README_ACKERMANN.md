
# LIMO Ackermann + Nav2 (ROS 2 Humble)

## What changed
- Removed `gazebo_ros_control` (ROS1-style) from `urdf/limo_ackerman.gazebo`
- Kept the Ackermann plugin (`libgazebo_ros_ackerman.so`) and normalized its plugin name to `ackermann_controller`
- Ensured parameters: `leftFrontJoint/front_right...`, steering hinges `left_steering_hinge_wheel`, `right_steering_hinge_wheel`, `wheelSeparation` and `wheelBase`
- Added consolidated launch: `launch/ackermann_gazebo_nav2.launch.py` to bring up Gazebo + RSP + spawn + Nav2
- Added quick check launch: `launch/ackermann_gazebo_only.launch.py`

## Run (Gazebo only, TF/odom check)
```bash
ros2 launch limo_description ackermann_gazebo_only.launch.py
# In another terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # publishes to /cmd_vel
# Inspect TF and odom:
ros2 run tf2_tools view_frames
rqt_graph
ros2 topic echo /odom --once
```

## Run (Gazebo + Nav2)
```bash
ros2 launch limo_description ackermann_gazebo_nav2.launch.py use_sim_time:=true slam:=False
# (uses bundled map/config by default)
# Optionally run teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Ensure `nav2_params.yaml` uses `base_frame_id:=base_link` and `odom_frame_id:=odom` (already set).

## Notes
- The Ackermann plugin publishes `/odom` and the `odom->base_footprint` TF. URDF has a fixed `base_footprint->base_link`, so Nav2's `base_link` frames chain correctly.
- If you don't have `libgazebo_ros_ackerman.so` installed, switch to `ros2_control` + `ackermann_steering_controller` path instead.
