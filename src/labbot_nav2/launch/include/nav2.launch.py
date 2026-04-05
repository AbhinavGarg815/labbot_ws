import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    nav2_params = os.path.join(
        get_package_share_directory('labbot_nav2'),
        'config', 'nav2_params.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ── Convert depth image → virtual laser scan ──────────────────────
        # Nav2 costmaps need a LaserScan. depthimage_to_laserscan slices a
        # thin horizontal band out of the depth image and converts it.
        # scan_height: number of pixel rows to collapse → thicker virtual beam
        # range_max: clip distant readings (D435i reliable range ~4 m in sim)
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'use_sim_time':  LaunchConfiguration('use_sim_time'),
                'scan_height':   10,
                'range_min':     0.15,
                'range_max':     4.0,
                'scan_time':     0.033,
                'output_frame':  'realsense_DCAM_1',
            }],
            remappings=[
                ('depth',        '/d435i_depth_camera/depth/image_raw'),
                ('depth_camera_info',  '/d435i_depth_camera/depth/camera_info'),
                ('scan',         '/scan'),
            ],
        ),

        # ── Nav2 stack ────────────────────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav'),
                ('odom', '/odometry/filtered'),
            ]
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel',        '/cmd_vel_nav'),
                ('cmd_vel_smoothed', '/diff_drive_controller/cmd_vel_unstamped'),
            ],
        ),

        # ── Lifecycle manager — activates all nav2 nodes in order ─────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[nav2_params],
        ),
    ])