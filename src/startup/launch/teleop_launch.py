from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    robot_description_arg = DeclareLaunchArgument(
        "selected_robot",
        default_value="training_bot",
        description="Robot description to load"
    )
    selected_robot = LaunchConfiguration("selected_robot")
    startup_pkg = FindPackageShare("startup")

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            startup_pkg,
            "robot_description",
            selected_robot,
            "urdf",
            "robot.urdf.xacro"
        ]),
        " use_sim:=",
        "false"
    ])

    controller_config = ParameterFile(
        PathJoinSubstitution([
        startup_pkg,
        "robot_description",
        selected_robot,
        "config",
        "controllers.yaml"
        ]),
        allow_substs=True
    )

    return LaunchDescription([
        robot_description_arg,

        # Publishes URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description}
            ],
        ),

        # Starts ros2_control and hardware interface
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description},
                controller_config
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