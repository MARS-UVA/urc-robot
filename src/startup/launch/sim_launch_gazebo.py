from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_arg = DeclareLaunchArgument(
        "selected_robot",
        default_value="training_bot",
        description="Robot description to load"
    )
    selected_robot = LaunchConfiguration("selected_robot")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Gazebo world to load"
    )
    world = LaunchConfiguration("world")

    startup_pkg = FindPackageShare("startup")

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            startup_pkg,
            "robots",
            selected_robot,
            "urdf",
            "robot.urdf.xacro"
        ]),
        " use_sim:=true",
        " sim_backend:=gazebo"
    ])

    return LaunchDescription([
        robot_description_arg,
        world_arg,

        # Starts Gazebo, which loads gz_ros2_control and therefore hosts the
        # controller_manager itself -- no separate ros2_control_node here.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                ])
            ),
            launch_arguments={"gz_args": ["-r ", world]}.items(),
        ),

        # Publishes URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": True}
            ],
        ),

        # Bridges Gazebo's clock so every node shares sim time
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),

        # Spawns the robot into Gazebo from the published description
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "/robot_description",
                "-name", "training_bot",
                "-z", "0.1"
            ],
            output="screen",
        ),

        # Publishes joint states so robot_state_publisher can build the TF tree
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager"
            ],
            output="screen",
        ),

        # Loads drive controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diff_drive_controller",
                "--controller-manager",
                "/controller_manager"
            ],
            output="screen",
        ),

        # # Starts serial communication node
        # Node(
        #     package="serial_communication",
        #     executable="serial_comms",  # TODO: CHANGE THIS NAME WHEN NODE IS ACTUALLY CREATED
        #     name="serial",
        #     output="screen",
        # )
    ])
