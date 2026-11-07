"""Launch the URC rover in Gazebo."""

import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


CAD_SPAWN_YAW = 0
CAD_YAW_IN_BASE = 0


def shutdown_on_failure(event, context):
    """Stop the launch if spawning the robot or a controller fails."""
    if event.returncode and not context.is_shutdown:
        return [Shutdown(reason='Robot or controller startup failed')]
    return []


def launch_gazebo(context):
    """Start Gazebo without a shell and supervise all of its forked processes."""
    args = ['-r', '-v', '1', '--render-engine',
            LaunchConfiguration('render_engine').perform(context)]
    if IfCondition(LaunchConfiguration('no_gui')).evaluate(context):
        args += ['-s', '--headless-rendering']
    world = PathJoinSubstitution([
        FindPackageShare('startup'), 'world', LaunchConfiguration('choose_map'),
    ]).perform(context)
    return [ExecuteProcess(
        cmd=[sys.executable, '-m', 'startup.gazebo_process', '--',
             FindExecutable(name='gz'), 'sim', *args, world],
        name='gazebo', output='screen',
        on_exit=Shutdown(reason='Gazebo exited'),
        sigterm_timeout='4', sigkill_timeout='2',
    )]


def launch_icp_odometry(context):
    """Estimate base motion directly from Gazebo snapshot clouds."""
    if not IfCondition(LaunchConfiguration('enable_lidar')).evaluate(context):
        return []
    return [Node(
        package='rtabmap_odom', executable='icp_odometry',
        name='icp_odometry', output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('startup'), 'config', 'icp_odometry.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('scan_cloud', '/unilidar/cloud'), ('odom', '/icp/odom')],
        on_exit=Shutdown(reason='ICP odometry exited'),
    )]


def launch_mapping(context):
    """Map using ICP odometry and the front RGB-D/LiDAR sensors."""
    if not IfCondition(LaunchConfiguration('enable_rtabmap')).evaluate(context):
        return []
    if not IfCondition(LaunchConfiguration('enable_lidar')).evaluate(context):
        raise RuntimeError('Gazebo RTAB-Map requires enable_lidar:=true for ICP odometry')
    rgbd = IfCondition(LaunchConfiguration('enable_rgbd')).evaluate(context)
    lidar = IfCondition(LaunchConfiguration('enable_lidar')).evaluate(context)
    if not (rgbd or lidar):
        raise RuntimeError('RTAB-Map requires enable_rgbd:=true or enable_lidar:=true')
    database = Path(LaunchConfiguration('rtabmap_database_path').perform(context)).expanduser()
    database.parent.mkdir(parents=True, exist_ok=True)
    nodes = [Node(
        package='rtabmap_slam', executable='rtabmap',
        name='rtabmap', namespace='rtabmap', output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('startup'), 'config', 'rtabmap.yaml']),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'database_path': str(database),
                'qos_odom': 2,
                'delete_db_on_start': True,
                'subscribe_depth': rgbd,
                'subscribe_rgb': rgbd,
                'subscribe_scan_cloud': lidar,
                'Grid/Sensor': ParameterValue('0' if lidar else '1', value_type=str),
                'Reg/Strategy': ParameterValue('0' if rgbd else '1', value_type=str),
            },
        ],
        remappings=[
            ('rgb/image', '/front_rgbd/color/image_raw'),
            ('depth/image', '/front_rgbd/depth/image_raw'),
            ('rgb/camera_info', '/front_rgbd/camera_info'),
            ('scan_cloud', '/unilidar/cloud'),
            ('odom', '/icp/odom'),
            ('map', '/map'),
        ],
        on_exit=Shutdown(reason='RTAB-Map exited'),
    )]
    # Match mars-jetson: mapping opens its UI unless running without windows.
    if (not IfCondition(LaunchConfiguration('no_gui')).evaluate(context) and
            IfCondition(LaunchConfiguration('enable_rtabmap_viz')).evaluate(context)):
        nodes.append(Node(
            package='rtabmap_viz', executable='rtabmap_viz',
            name='rtabmap_viz', namespace='rtabmap', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'subscribe_depth': rgbd,
                'subscribe_rgb': rgbd,
                'subscribe_scan_cloud': lidar,
                'subscribe_odom_info': False,
                'approx_sync': True,
                'topic_queue_size': 30,
                'sync_queue_size': 30,
                'qos_image': 2,
                'qos_camera_info': 2,
                'qos_scan': 2,
                'qos_odom': 2,
            }],
            remappings=[
                ('rgb/image', '/front_rgbd/color/image_raw'),
                ('depth/image', '/front_rgbd/depth/image_raw'),
                ('rgb/camera_info', '/front_rgbd/camera_info'),
                ('scan_cloud', '/unilidar/cloud'),
                ('odom', '/icp/odom'),
            ],
        ))
    return nodes


def generate_launch_description():
    """Create the Gazebo launch description for the URC rover."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urc-robot',
        'urdf',
        'robot.urdf.xacro',
    ])
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' ',
                robot_description_file,
                ' use_sim:=true',
                ' enable_rgbd:=', LaunchConfiguration('enable_rgbd'),
                ' enable_side_cameras:=', LaunchConfiguration('enable_side_cameras'),
                ' enable_lidar:=', LaunchConfiguration('enable_lidar'),
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        on_exit=Shutdown(reason='Robot state publisher exited'),
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'urc-robot',
            '-allow_renaming', 'true',
            '-x', '1.00',
            '-y', '-1.50',
            '-z', '0.40',
            '-Y', str(CAD_SPAWN_YAW - CAD_YAW_IN_BASE),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        on_exit=shutdown_on_failure,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        on_exit=shutdown_on_failure,
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        on_exit=Shutdown(reason='Clock bridge exited'),
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(IfElseSubstitution(
            LaunchConfiguration('no_gui'), if_value='false',
            else_value=LaunchConfiguration('enable_rviz'),
        )),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'choose_map', default_value='empty.world',
            description='Gazebo world filename: empty.world or rock.world',
        ),
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
            'enable_rviz',
            default_value='false',
            description='Also launch RViz',
        ),
        DeclareLaunchArgument(
            'enable_rtabmap', default_value='true',
            description='Run RTAB-Map with the enabled RGB-D and LiDAR sensors',
        ),
        DeclareLaunchArgument(
            'enable_rtabmap_viz', default_value='true',
            description='Open the RTAB-Map UI when mapping is enabled and no_gui is false',
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=IfElseSubstitution(
                LaunchConfiguration('enable_rtabmap'),
                if_value=PathJoinSubstitution([
                    FindPackageShare('startup'), 'config', 'mapping.rviz',
                ]),
                else_value=PathJoinSubstitution([
                    FindPackageShare('startup'), 'config', 'sensors.rviz',
                ]),
            ),
            description='RViz configuration for mapping or navigation',
        ),
        DeclareLaunchArgument(
            'rtabmap_database_path', default_value=str(Path.home() / '.ros/rtabmap/urc_gazebo.db'),
            description='RTAB-Map database cleared at each simulation launch',
        ),
        DeclareLaunchArgument(
            'enable_rgbd', default_value='true',
            description='Spawn and bridge the front RGB-D camera',
        ),
        DeclareLaunchArgument(
            'enable_lidar', default_value='true',
            description='Spawn and bridge the rear roof 360-degree LiDAR',
        ),
        DeclareLaunchArgument('enable_side_cameras', default_value='true',
                              description='Spawn and bridge the left and right RGB cameras'),
        SetEnvironmentVariable(
            name='GZ_IP',
            value=EnvironmentVariable(name='GZ_IP', default_value='127.0.0.1'),
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                PathJoinSubstitution([FindPackageShare('startup'), 'models']),
                ':',
                PathJoinSubstitution([
                    FindPackageShare('robot_description'),
                    '..',
                ]),
            ],
        ),
        *[
            Node(
                package='ros_gz_bridge', executable='parameter_bridge',
                name=name + '_bridge', output='screen',
                on_exit=Shutdown(reason=name + ' bridge exited'),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'config_file': PathJoinSubstitution([
                        FindPackageShare('startup'), 'config', name + '_bridge.yaml',
                    ]),
                }],
                condition=IfCondition(LaunchConfiguration(flag)),
            )
            for name, flag in [
                ('front_rgbd', 'enable_rgbd'),
                ('side_cameras', 'enable_side_cameras'),
                ('lidar', 'enable_lidar'),
            ]
        ],
        bridge,
        OpaqueFunction(function=launch_gazebo),
        OpaqueFunction(function=launch_icp_odometry),
        OpaqueFunction(function=launch_mapping),
        robot_state_publisher,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=lambda event, context: (
                    [] if context.is_shutdown else
                    shutdown_on_failure(event, context) if event.returncode else
                    [joint_state_broadcaster_spawner, robot_controller_spawner]
                ),
            )
        ),
        gz_spawn_entity,
        rviz,
    ])
