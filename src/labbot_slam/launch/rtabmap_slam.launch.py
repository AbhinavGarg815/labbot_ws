import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    description_pkg = get_package_share_directory('labbot_description')
    description_launch_dir = os.path.join(description_pkg, 'launch')

    slam_pkg = get_package_share_directory('labbot_slam')
    slam_launch_dir = os.path.join(slam_pkg, 'launch')

    # ── Arguments ──────────────────────────────────────────────────────────
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    slam_delay = DeclareLaunchArgument(
        'slam_delay',
        default_value='8.0',
        description='Seconds to wait for Gazebo + controllers before starting RTAB-Map'
    )

    localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Start RTAB-Map in localisation mode (true) or mapping mode (false)'
    )

    rtabmap_viz = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='true',
        description='Launch RTAB-Map visualiser'
    )

    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    qos = DeclareLaunchArgument(
        'qos',
        default_value='2',
        description='QoS reliability: 1=reliable, 2=best_effort'
    )

    # ── Tier 1: Simulation ─────────────────────────────────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # ── Tier 2: RTAB-Map — delayed until Gazebo + controllers are ready ────
    slam_launch = TimerAction(
        period=LaunchConfiguration('slam_delay'),
        actions=[
            GroupAction(actions=[
                LogInfo(msg='====== Starting RTAB-Map SLAM ======'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_launch_dir, 'include/rtabmap.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time':             LaunchConfiguration('use_sim_time'),
                        'localization':             LaunchConfiguration('localization'),
                        'rtabmap_viz':              LaunchConfiguration('rtabmap_viz'),
                        'rviz':                     LaunchConfiguration('rviz'),
                        'qos':                      LaunchConfiguration('qos'),
                        'approx_sync':              'true',
                        'approx_sync_max_interval': '0.02',
                    }.items()
                ),
            ])
        ]
    )

    return LaunchDescription([
        # arguments first
        use_sim_time,
        slam_delay,
        localization,
        rtabmap_viz,
        rviz,
        qos,

        # tier 1
        LogInfo(msg='====== Starting Gazebo Simulation ======'),
        sim_launch,

        # tier 2
        slam_launch,
    ])