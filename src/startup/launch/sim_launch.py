"""Launch the URC rover in Gazebo."""

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


CAD_SPAWN_YAW = math.pi / 2
CAD_YAW_IN_BASE = math.pi / 2


def generate_launch_description():
    """Create the Gazebo launch description for the URC rover."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    no_gui = LaunchConfiguration('no_gui')

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
            ]),
            value_type=str,
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
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'no_gui',
            default_value='false',
            description='Run Gazebo without its GUI.',
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='Also launch RViz',
        ),
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
        bridge,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])),
            launch_arguments={
                'gz_args': [
                    IfElseSubstitution(
                        no_gui,
                        if_value='-s --headless-rendering -r -v 1 ',
                        else_value='-r -v 1 ',
                    ),
                    PathJoinSubstitution([
                        FindPackageShare('startup'),
                        'world',
                        'empty.world',
                    ]),
                ],
            }.items(),
        ),
        robot_state_publisher,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    robot_controller_spawner,
                ],
            )
        ),
        gz_spawn_entity,
        rviz,
    ])
