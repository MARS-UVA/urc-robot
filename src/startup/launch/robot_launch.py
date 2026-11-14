"""Launch the URC rover with its real hardware control system."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the hardware-control launch description."""
    selected_robot_arg = DeclareLaunchArgument(
        'selected_robot',
        default_value='urc-robot',
        description='Robot description to load',
    )
    controller_config_arg = DeclareLaunchArgument(
        'controller_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('rocker_bogie_control'),
            'config',
            'ros2_controllers.yaml',
        ]),
        description='ROS 2 controller configuration file',
    )

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('robot_description'),
            LaunchConfiguration('selected_robot'),
            'urdf',
            'robot.urdf.xacro',
        ]),
        ' use_sim:=false',
    ])
    controller_config = ParameterFile(
        LaunchConfiguration('controller_config'),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_point_lio', default_value='true',
                              description='Run Point-LIO using real LiDAR/IMU data'),
        DeclareLaunchArgument('point_lio_config', default_value=PathJoinSubstitution([
            FindPackageShare('point_lio'), 'config', 'unilidar_l2.yaml',
        ])),
        DeclareLaunchArgument('point_lio_sensor_frame', default_value='unilidar_imu'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('startup'), 'launch', 'point_lio_launch.py',
            ])),
            condition=IfCondition(LaunchConfiguration('enable_point_lio')),
            launch_arguments={
                'point_lio_config': LaunchConfiguration('point_lio_config'),
                'point_lio_sensor_frame': LaunchConfiguration('point_lio_sensor_frame'),
            }.items(),
        ),
        selected_robot_arg,
        controller_config_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controller_config,
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager',
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager',
                '/controller_manager',
            ],
            output='screen',
        ),
        Node(
            package='aruco_detection',
            executable='aruco_detection',
            output='screen'
        )
    ])
