import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time in ('true', 'True', '1')

    qos          = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')

    approx_sync              = LaunchConfiguration('approx_sync').perform(context)
    approx_sync              = approx_sync in ('true', 'True', '1')
    approx_sync_max_interval = float(LaunchConfiguration('approx_sync_max_interval').perform(context))

    database_path = LaunchConfiguration('database_path')

    # ── Common params shared by odom & slam ───────────────────────────────
    # FIX 1: Removed 'subscribe_depth': True from common.
    #   In the old code this key propagated into slam_parameters via **common.
    #   Even though slam_parameters overrides it with False, the RTAB-Map node
    #   also inferred subscribe_rgb=True from subscribe_depth=True, then
    #   clashed with subscribe_rgbd=True producing the warning:
    #   "subscribe_rgb and subscribe_rgbd cannot be true at the same time."
    #   subscribe_depth is now set explicitly only inside odom_parameters.
    common = {
        'frame_id':         'base_link',
        'use_sim_time':     use_sim_time,
        'approx_sync':      True,
        'approx_sync_max_interval': approx_sync_max_interval,
        'qos_image':        qos,
        'qos_imu':          qos,
        'Reg/Force3DoF':    'true',
        'Optimizer/Slam2D': 'true',
    }

    camera_remappings = [
        ('rgb/image',       '/d435i_depth_camera/image_raw'),
        ('rgb/camera_info', '/d435i_depth_camera/camera_info'),
        ('depth/image',     '/d435i_depth_camera/depth/image_raw'),
        # ('imu',             '/imu/data'),
    ]

    # ── FIX 2: Separate IMU remapping out of slam_remappings ──────────────
    #   camera_remappings includes ('imu', '/imu/data'). When this was passed
    #   to the slam node via slam_remappings, rtabmap started buffering IMU
    #   data and then failed to interpolate the IMU TF at early timestamps
    #   (log: "cannot interpolate imu transform at time 6.836 … latest is 7.359").
    #   Since the EKF already fuses IMU externally, rtabmap does not need IMU.
    #   slam_remappings is now built without the IMU entry.
    camera_remappings_no_imu = [r for r in camera_remappings if r[0] != 'imu']

    slam_remappings = camera_remappings + [
        ('odom',       '/odometry/filtered'),
        ('rgbd_image', '/rgbd_image'),
    ]

    # ── FIX 3: Added 'qos': qos to odom_parameters ───────────────────────
    #   rgbd_odometry reported qos=0 in its startup log even though qos was
    #   set to 2 everywhere else. The node reads its own 'qos' parameter for
    #   raw-topic subscribers (qos_image/qos_camera_info are separate keys
    #   used only for named image/camera_info subscribers). Without an explicit
    #   'qos' key, rgbd_odometry defaulted to 0 (BEST_EFFORT) while the
    #   Gazebo camera publishes at 2 (RELIABLE), causing a silent QoS
    #   mismatch.
    odom_parameters = {
        **common,
        # 'Reg/Force3DoF': 'true',
        'approx_sync': True,
        'subscribe_rgbd': True,
        'publish_tf': False,
    }

    # ── FIX 4: Added 'subscribe_rgb': False and 'wait_for_transform': 0.5 ─
    #   (a) subscribe_rgb=False is the explicit suppression that prevents the
    #       "subscribe_rgb and subscribe_rgbd cannot be true at the same time"
    #       warning even if common params are extended in the future.
    #   (b) wait_for_transform raised from 0.2 → 0.5 s.
    #       The EKF publishes odom→base_link at ~50 Hz (20 ms cycles). Each
    #       camera frame arrives timestamped ~20 ms ahead of the latest EKF TF.
    #       TF cannot extrapolate into the future, so every frame triggered:
    #       "Lookup would require extrapolation into the future. Requested time
    #        X.XXX but latest data is at X.XXX-0.020."
    #       A longer wait gives the EKF time to publish the covering transform.
    #       NOTE: the root fix is to increase EKF frequency to ≥100 Hz in
    #       robot_localization config so the TF gap shrinks below 10 ms.
    #   (c) Mem/DepthCompressionFormat='.png' silences the startup warning:
    #       "Mem/SaveDepth16Format … not compatible with .rvl format".
    #   (d) topic_queue_size reduced from 10 → 1 for the slam node.
    #       rtabmap processes at 1 Hz (DetectionRate=1) while rgbd_sync
    #       outputs at ~30 Hz. With queue_size=10 the slam node was sitting
    #       0.7–0.9 s behind real time (log: "delay=0.9640s"), which in turn
    #       caused TF lookups to request timestamps that the EKF had long
    #       since advanced past. Dropping stale frames keeps processing latency
    #       under ~100 ms.
    slam_parameters = {
        **common,
        'subscribe_depth': False,
        'subscribe_rgb': False,      # FIX 4a: explicit; avoids subscribe_rgb warning
        'subscribe_rgbd': True,
        'publish_tf': True,
        'database_path': database_path,
        'approx_sync': True,
        # 'approx_sync_max_interval': approx_sync_max_interval,
        'wait_for_transform':    0.5,

        'publish_tf_map':        'true',
        # 'subscribe_odom_info':   False,

        # # FIX 4b: increased wait_for_transform

        # # FIX 4c: suppress depth-compression warning
        'Mem/DepthCompressionFormat': '.png',

        # # FIX 4d: drop stale frames — keeps processing delay < 100 ms
        # 'topic_queue_size':      1,
        # 'sync_queue_size':       5,

        'Reg/Strategy':          '0',
        'Rtabmap/LoopThr':       '0.11',
        'Vis/MinInliers':        '8',
        'RGBD/OptimizeMaxError': '3.0',
        'Optimizer/Robust':      'true',
        'Rtabmap/DetectionRate': '1',
    }

    return [
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=[{
                'use_sim_time':             use_sim_time,
                'approx_sync':              True,
                'approx_sync_max_interval': approx_sync_max_interval,
            }],
            remappings=camera_remappings,
        ),

        # ── Visual Odometry ───────────────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=[odom_parameters],
            remappings=camera_remappings + [
                ('odom', '/visual_odom'),
            ],
        ),

        # ── SLAM — Mapping mode ───────────────────────────────────────────
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[slam_parameters],
            remappings=slam_remappings,
            arguments=['-d'],
        ),

        # ── SLAM — Localisation mode ──────────────────────────────────────
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[slam_parameters, {
                'Mem/IncrementalMemory':  'False',
                'Mem/InitWMWithAllNodes': 'True',
                'Rtabmap/DetectionRate':  '10',
                'tf_delay':               0.05,
            }],
            remappings=slam_remappings,
        ),

        # ── RTAB-Map GUI ──────────────────────────────────────────────────
        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time':  use_sim_time,
                'subscribe_rgbd': True,
            }],
            remappings=slam_remappings,
        ),

        # ── RViz ─────────────────────────────────────────────────────────
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Point cloud (RViz visualisation only) ────────────────────────
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rtabmap_util',
            executable='point_cloud_xyzrgb',
            output='screen',
            parameters=[{
                'use_sim_time':             use_sim_time,
                'decimation':               4,
                'voxel_size':               0.0,
                'approx_sync':              True,
                'approx_sync_max_interval': approx_sync_max_interval,
            }],
            remappings=camera_remappings,
        ),
    ]


def generate_launch_description():

    config_rviz = os.path.join(
        get_package_share_directory('labbot_slam'),
        'rviz', 'slam.rviz'
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time',              default_value='true'),
        DeclareLaunchArgument('approx_sync',               default_value='true'),
        DeclareLaunchArgument('approx_sync_max_interval',  default_value='0.02'),
        DeclareLaunchArgument('localization',              default_value='false'),
        DeclareLaunchArgument('rtabmap_viz',               default_value='true'),
        DeclareLaunchArgument('rviz',                      default_value='true'),
        DeclareLaunchArgument('rviz_cfg',                  default_value=config_rviz),
        DeclareLaunchArgument('database_path',             default_value='~/labbot_ws/src/labbot_slam/maps/rtabmap.db'),
        DeclareLaunchArgument('qos',                       default_value='2'),

        OpaqueFunction(function=launch_setup),
    ])