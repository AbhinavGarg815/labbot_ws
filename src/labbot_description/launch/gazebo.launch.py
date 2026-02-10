from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Configuration
    robot_xacro_name = 'labbot'
    package_name = 'labbot_description'
    model_file_rel_path = 'urdf/labbot.xacro'
    world_file_abs_path = '/usr/share/gazebo-11/worlds/empty.world'  # Absolute path

    # Resolve model file path
    package_share = get_package_share_directory(package_name)
    path_model_file = os.path.join(package_share, model_file_rel_path)
    path_world_file = world_file_abs_path  # Already absolute

    # File existence checks
    if not os.path.exists(path_model_file):
        raise FileNotFoundError(f"Xacro file not found: {path_model_file}")
    if not os.path.exists(path_world_file):
        raise FileNotFoundError(f"World file not found: {path_world_file}")

    # Process xacro to URDF
    robot_description = xacro.process_file(path_model_file).toxml()

    # Include Gazebo launch
    gazebo_launch_source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_launch_source,
        launch_arguments={'world': path_world_file}.items()
    )

    # Spawn the robot in Gazebo
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_xacro_name, '-topic', 'robot_description'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # controller_params_file = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     'controllers.yaml'
    # )

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {'robot_description': robot_description},
    #         controller_params_file
    #     ],
    #     output='screen',
    # )

    # delayed_controller_manager = TimerAction(
    #     period=3.0,
    #     actions=[controller_manager]
    # )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_model_node,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=diff_drive_spawner,
            on_start=[joint_broad_spawner],
        )
    )

    camera_position_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["camera_position_controller", "--controller-manager", "/controller_manager"],
        output='screen',
    )

    # Chain it after diff_drive_controller starts (or after joint_state if you prefer)
    delayed_camera_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=diff_drive_spawner,  # or joint_broad_spawner if you want even earlier
            on_start=[camera_position_spawner],
        )
    )

    # Build and return the launch description
    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_model_node)
    ld.add_action(robot_state_publisher_node)
    # ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_diff_drive_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(delayed_camera_spawner)     

    return ld