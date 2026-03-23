# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
#   RTAB-Map in mapping mode

import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from typing import Text
from ament_index_python.packages import get_package_share_directory


class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> Text:
        if self.condition in (True, 'true', 'True'):
            return self.text_if
        else:
            return self.text_else


def launch_setup(context, *args, **kwargs):

    use_sim_time = True
    qos          = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    # ─────────────────────────────────────────────────────────────────────────
    # COMMON: shared low-level settings (frame, sync, topics)
    # ─────────────────────────────────────────────────────────────────────────
    common = {
        'frame_id':         'base_link',
        'use_sim_time':     use_sim_time,
        'subscribe_depth':  True,
        'approx_sync':      True,
        'qos_image':        qos,
        'qos_imu':          qos,
        'Reg/Force3DoF':    'true',      # planar robot
        'Optimizer/Slam2D': 'true',
    }
 
    # ─────────────────────────────────────────────────────────────────────────
    # VISUAL ODOMETRY
    # ─────────────────────────────────────────────────────────────────────────
    odom_parameters = {
        **common,
        'Reg/Strategy':   '0',       # visual only — faster for odometry
        'Odom/Strategy':  '0',       # Frame-to-Map
        'publish_tf':     False,     # EKF owns the odom→base_link TF
    }
 
    # ─────────────────────────────────────────────────────────────────────────
    # SLAM
    # ─────────────────────────────────────────────────────────────────────────
    slam_parameters = {
        **common,
        'qos_odom':              qos,
        'odom_frame_id':         'odom',
        'publish_tf':            True,       # publishes map→odom TF
        'publish_tf_map':        'true',
        'subscribe_odom_info':   False,
 
        # Sim time tolerance
        'approx_sync_max_interval': '0.1',
 
        # Registration — visual only (ICP needs better depth than Gazebo provides)
        'Reg/Strategy':          '0',
 
        # Loop closure
        'Rtabmap/LoopThr':       '0.11',
        'Vis/MinInliers':        '8',
 
        # Keep map corrections smooth rather than jumping
        'RGBD/OptimizeMaxError': '3.0',
        'Optimizer/Robust':      'true',
 
        'Rtabmap/DetectionRate': '1',    # 1 Hz is enough for mapping
    }

    # ─────────────────────────────────────────────────────────────────────────
    # Topic remappings
    # ─────────────────────────────────────────────────────────────────────────
    camera_remappings = [
        ('rgb/image',        '/d435i_depth_camera/image_raw'),
        ('rgb/camera_info',  '/d435i_depth_camera/camera_info'),
        ('depth/image',      '/d435i_depth_camera/depth/image_raw'),
        ('imu',              '/imu/data'),
    ]

    slam_remappings = camera_remappings + [
        ('odom', '/odometry/filtered'),
    ]

    return [

        # ── Visual Odometry ───────────────────────────────────────────────
        # Publishes /visual_odom → EKF fuses it with wheel encoders
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[odom_parameters, {'publish_tf': False}],
            remappings=camera_remappings + [
                ('odom', '/visual_odom'),   # goes to EKF, not directly to SLAM
            ]
        ),

        # ── SLAM — Mapping mode ───────────────────────────────────────────
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[slam_parameters],
            remappings=slam_remappings,
            arguments=['-d'],
        ),

        # ── SLAM — Localisation mode ──────────────────────────────────────
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[slam_parameters, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True',
            }],
            remappings=slam_remappings,
        ),

        # ── RTAB-Map GUI ──────────────────────────────────────────────────
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[slam_parameters, {'publish_tf': False}],
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            remappings=slam_remappings,
        ),

        # ── RViz ─────────────────────────────────────────────────────────
        Node(
            package='rviz2', executable='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=[['-d'], [LaunchConfiguration('rviz_cfg')]],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Point cloud (RViz visualisation only) ────────────────────────
        Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{
                'use_sim_time': use_sim_time,
                'decimation':             4,
                'voxel_size':             0.0,
                'approx_sync':            LaunchConfiguration('approx_sync'),
                'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
            }],
            remappings=camera_remappings,
        ),
    ]


def generate_launch_description():

    config_rviz = os.path.join(
        get_package_share_directory('labbot_description'), 'rviz', 'labbot_odom.rviz')

    return LaunchDescription([

        DeclareLaunchArgument('approx_sync', default_value='false'),
        DeclareLaunchArgument('approx_sync_max_interval', default_value='0.0'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz),
        DeclareLaunchArgument('initial_pose', default_value=''),
        DeclareLaunchArgument('rtabmap_args', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('qos', default_value='2'),

        OpaqueFunction(function=launch_setup),
    ])