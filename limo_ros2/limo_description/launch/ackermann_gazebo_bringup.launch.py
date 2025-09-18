from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    use_sim_time = True
    pkg = get_package_share_directory('limo_description')
    world = os.path.join(pkg, 'worlds', 'empty_limo.world')
    robot_xacro = os.path.join(pkg, 'urdf', 'limo_ackerman.xacro')

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # URDF -> robot_description (xacro 실행을 FindExecutable로 안전하게)
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), robot_xacro]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description_content}],
        output='screen'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'limo'],
        output='screen'
    )

    controllers_yaml = os.path.join(pkg, 'config', 'ros2_controllers.yaml')

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    ack = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers_yaml]
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(pkg, 'config', 'twist_mux.yaml')]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav2_basic.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([gazebo, rsp, spawn, jsb, ack, twist_mux, rviz])
