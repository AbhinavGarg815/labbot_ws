from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_xacro_name = 'labbot'
    package_name     = 'labbot_description'
    package_share    = get_package_share_directory(package_name)

    path_model_file = os.path.join(package_share, 'urdf', 'labbot.xacro')
    path_world_file = os.path.join(package_share, 'worlds', 'cafe.world')
    gazebo_params   = os.path.join(package_share, 'config', 'gazebo.yaml')

    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(package_share, 'models')

    if not os.path.exists(path_model_file):
        raise FileNotFoundError(f"Xacro file not found: {path_model_file}")
    if not os.path.exists(path_world_file):
        raise FileNotFoundError(f"World file not found: {path_world_file}")

    robot_description = xacro.process_file(path_model_file).toxml()

    # ── Gazebo ──────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':       path_world_file,
            'params_file': gazebo_params,
        }.items()
    )

    # ── Robot State Publisher ───────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # ── Spawn robot ─────────────────────────────────────────────────
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_xacro_name,
                   '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.0625'],
        output='screen'
    )

    # ── Controllers — chained via OnProcessExit ─────────────────────
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
    )

    camera_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['camera_position_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(package_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
        ]
    )

    # spawn_entity exits → start joint_state_broadcaster
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_model_node,
            on_exit=[joint_broad_spawner],
        )
    )

    # joint_state_broadcaster exits → start diff_drive
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # diff_drive exits → start camera controller + EKF together
    delayed_camera_and_ekf = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[camera_position_spawner, ekf_node],
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_model_node,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        delayed_camera_and_ekf,
    ])