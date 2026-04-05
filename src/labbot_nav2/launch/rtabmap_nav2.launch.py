"""
navigation_bringup.launch.py
────────────────────────────
Single launch file that brings up everything needed for autonomous navigation:

  Terminal 1 (simulation):
      ros2 launch labbot_description gazebo.launch.py

  Terminal 2 (this file):
      ros2 launch labbot_slam navigation_bringup.launch.py

  Then in RViz:
      • Add display: Map  (topic: /map)
      • Add display: Nav2 → Navigation2 plugin  OR
        use the "2D Nav Goal" button on the toolbar to send a goal pose
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_pkg = get_package_share_directory('labbot_slam')
    navigation_pkg = get_package_share_directory('labbot_nav2')

    rtabmap_launch = os.path.join(slam_pkg, 'launch', 'rtabmap_slam.launch.py')
    nav2_launch    = os.path.join(navigation_pkg, 'launch/include', 'nav2.launch.py')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ── Step 1: RTAB-Map in localisation mode ─────────────────────────
        # localization:=true → loads the saved .db, publishes map → odom TF
        # and /map topic consumed by Nav2's global costmap static layer.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                'localization':  'true',
                'use_sim_time':  LaunchConfiguration('use_sim_time'),
                'rtabmap_viz':   'false',   # save resources; use RViz instead
                'rviz':          'true',
            }.items(),
        ),

        # ── Step 2: Nav2 (delayed 8 s to let RTAB-Map publish /map first) ─
        # The static_layer in the global costmap subscribes to /map at startup.
        # If Nav2 starts before RTAB-Map has published the first map message,
        # the global costmap initialises empty and navigation never works.
        TimerAction(
            period=20.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }.items(),
                ),
            ],
        ),
    ])