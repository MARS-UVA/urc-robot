from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, EnvironmentVariable, FindExecutable, IfElseSubstitution,
    LaunchConfiguration, NotSubstitution, PathJoinSubstitution,
)

from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import math


# Gazebo spawns the URDF root (blue-forward base_link). Preserve the original
# CAD frame_assembly world yaw despite its fixed +pi/2 joint from base_link.
CAD_SPAWN_YAW = 1.5708
CAD_YAW_IN_BASE = math.pi / 2


def launch_rtabmap(context):
    if not IfCondition(LaunchConfiguration('enable_rtabmap')).evaluate(context):
        return []
    enable_rgbd = IfCondition(LaunchConfiguration('enable_rgbd')).evaluate(context)
    enable_lidar = IfCondition(LaunchConfiguration('enable_lidar')).evaluate(context)
    if not (enable_rgbd or enable_lidar):
        raise RuntimeError('enable_rtabmap requires enable_rgbd:=true or enable_lidar:=true')

    sync_rgbd = IfCondition(LaunchConfiguration('rtabmap_sync')).evaluate(context)
    if sync_rgbd and not enable_rgbd:
        raise RuntimeError('rtabmap_sync requires enable_rgbd:=true')

    # Resolve the optional package only when RTAB-Map is requested.
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py',
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'base_link',
            'visual_odometry': 'false' if enable_lidar else 'true',
            'icp_odometry': 'true' if enable_lidar else 'false',
            'depth': 'true' if enable_rgbd and not sync_rgbd else 'false',
            'subscribe_rgb': 'true' if enable_rgbd and not sync_rgbd else 'false',
            'subscribe_rgbd': 'true' if sync_rgbd else 'false',
            'rgbd_sync': 'false',
            'subscribe_scan_cloud': 'true' if enable_lidar else 'false',
            'scan_cloud_topic': '/unilidar/cloud',
            'publish_tf_odom': 'true',
            'vo_frame_id': 'odom',
            'odom_topic': '/rtabmap/icp_odom' if enable_lidar else '/rtabmap/visual_odom',
            'rtabmap_args': (
                '--Reg/Strategy 1 --Reg/Force3DoF true '
                '--RGBD/ProximityBySpace true --Grid/Sensor 0 '
                # Flat arena: wheel radius 0.1778 - axle Z 0.04445 gives
                # ground Z ~-0.133 m in base_link. Allow ~5 cm floor tolerance.
                '--Grid/NormalsSegmentation false --Grid/MaxGroundHeight -0.08 '
                '--Grid/MapFrameProjection false --Grid/GroundIsObstacle false '
                # Remove body returns from occupancy grids, in blue-forward base_link axes.
                # Use the unpadded envelope; retain obstacles outside the robot.
                '--Grid/FootprintLength 1.80 --Grid/FootprintWidth 0.76 '
                '--Grid/FootprintHeight 0.50 '
                '--Icp/VoxelSize 0.05 --Icp/PointToPlane true'
                if enable_lidar else ''
            ) + (' --Vis/EstimationType 0' if sync_rgbd else ''),
            'rgb_topic': '/rgbd_blue/color/image_raw',
            'depth_topic': '/rgbd_blue/depth/image_raw',
            'camera_info_topic': '/rgbd_blue/camera_info',
            'approx_sync': 'true',
            'qos': '2',
            'rtabmap_viz': NotSubstitution(LaunchConfiguration('no_gui', default='false')),
            'rviz': 'false',
        }.items(),
    )
    if sync_rgbd:
        # The upstream launch does not expose RGBDImages input arguments.
        # Apply these to mapping, visualization and optional RGB-D odometry.
        return [GroupAction(actions=[
            SetParameter(name='rgbd_cameras', value=0),
            SetRemap(src='rgbd_images', dst='/rgbd_images'),
            rtabmap,
        ])]
    return [rtabmap]


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    robot_description_file = PathJoinSubstitution([
    FindPackageShare('robot_description'),
        'bruno',
        'urdf',
        'robot.urdf.xacro',
    ])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' ',
                robot_description_file,
            ]),
            value_type=str
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Nodes important for running the simulation and controlling the robot
    # Center the robot in the assumed 2 x 2 m starting zone.
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'Bruno', '-allow_renaming', 'true',
                    '-x', '1.00',
                    '-y', '-1.50',
                    '-z', '0.12',
                    '-Y', str(CAD_SPAWN_YAW - CAD_YAW_IN_BASE),],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/unilidar/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/unilidar/cloud/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/unilidar/cloud/points', '/unilidar/cloud')
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/front_camera/image_raw', '/rear_camera/image_raw'],
        parameters=[{'qos': 'sensor_data', 'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('robot_description'), 'bruno', 'config', 'robot.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('no_gui')),
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'render_engine',
            default_value='ogre' if os.environ.get('LIBGL_ALWAYS_SOFTWARE') == '1' else 'ogre2',
            description='Gazebo renderer; use ogre for software rendering in the container',
        ),
        DeclareLaunchArgument(
            'no_gui', default_value='false',
            description='Run simulation without Gazebo, RViz, or RTAB-Map windows',
        ),
        DeclareLaunchArgument(
            'enable_rviz', default_value=NotSubstitution(LaunchConfiguration('no_gui')),
            description='Show the RGB-D or navigation RViz view',
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([
                FindPackageShare('robot_description'), 'bruno', 'config', 'RGBDRViz.rviz',
            ]),
            description='RViz configuration for mapping or navigation',
        ),
        DeclareLaunchArgument(
            'enable_rtabmap', default_value='false',
            description='Run RTAB-Map with the enabled RGB-D and LiDAR sensors',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path', default_value='/tmp/rgbd_visual_test.db',
            description='RTAB-Map database to save or resume',
        ),
        DeclareLaunchArgument(
            'enable_rgbd', default_value=LaunchConfiguration('enable_rtabmap'),
            description='Bridge the optional blue and orange RGB-D cameras (also enable it in robot_description)',
        ),
        DeclareLaunchArgument(
            'enable_lidar', default_value=LaunchConfiguration('enable_rtabmap'),
            description='Bridge the optional simulated LiDAR (also enable it in robot_description)',
        ),
        DeclareLaunchArgument(
            'rtabmap_sync', default_value='false',
            description='Synchronize blue and orange RGB-D cameras into /rgbd_images',
        ),
        OpaqueFunction(function=launch_rtabmap),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('startup'), 'launch', 'rgbd_sim.launch.py',
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz_config': LaunchConfiguration('rviz_config'),
                'enable_rviz': LaunchConfiguration('enable_rviz'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_rgbd')),
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[PathJoinSubstitution([FindPackageShare('startup'), 'models'])]
        ),
        bridge,
        lidar_bridge,
        camera_bridge,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [
                    ' --render-engine ', LaunchConfiguration('render_engine'),
                    IfElseSubstitution(
                        LaunchConfiguration('no_gui'),
                        if_value=' -s --headless-rendering -r -v 1 ',
                        else_value=' -r -v 1 ',
                    ),
                    PathJoinSubstitution([
                        FindPackageShare('startup'),
                        'world',
                        'empty.world'
                    ])
            ])]
        ),
        robot_state_publisher,
        gz_spawn_entity,
        rviz,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
    return ld
