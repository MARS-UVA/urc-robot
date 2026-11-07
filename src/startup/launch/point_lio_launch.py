"""Real-sensor Point-LIO odometry; sensor drivers and mount TF run separately."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_point_lio(context):
    """Use native point timing and the real IMU."""
    Path('/tmp/urc_point_lio/Log').mkdir(parents=True, exist_ok=True)
    sensor_frame = LaunchConfiguration('point_lio_sensor_frame').perform(context)
    return [
        Node(
            package='point_lio', executable='pointlio_mapping',
            name='point_lio', namespace='point_lio', output='screen',
            parameters=[LaunchConfiguration('point_lio_config'), {
                'use_sim_time': False,
                'odom_only': True,
                'odom_header_frame_id': 'point_lio_odom',
                'odom_child_frame_id': sensor_frame,
                'runtime_pos_log_enable': False,
                'publish.path_en': False,
                'publish.scan_publish_en': False,
                'pcd_save.pcd_save_en': False,
            }],
            remappings=[('/odom_corrected', '/point_lio/raw_odom'),
                        ('/tf', '/point_lio/raw_tf'),
                        ('/tf_static', '/point_lio/raw_tf_static')],
            on_exit=Shutdown(reason='Point-LIO exited'),
        ),
        Node(
            package='startup', executable='point_lio_odometry', output='screen',
            parameters=[{'use_sim_time': False, 'sensor_frame': sensor_frame}],
            on_exit=Shutdown(reason='Point-LIO base odometry adapter exited'),
        ),
    ]


def generate_launch_description():
    """Default to the Unitree L2 profile used by mars-jetson."""
    return LaunchDescription([
        DeclareLaunchArgument('point_lio_config', default_value=PathJoinSubstitution([
            FindPackageShare('point_lio'), 'config', 'unilidar_l2.yaml',
        ]), description='Real LiDAR/IMU profile, including calibrated extrinsics and timing'),
        DeclareLaunchArgument('point_lio_sensor_frame', default_value='unilidar_imu',
                              description='IMU pose frame with a calibrated TF to base_link'),
        OpaqueFunction(function=launch_point_lio),
    ])
