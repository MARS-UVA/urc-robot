from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Load the Skeleton URDF from this teleop package
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="cat")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("teleop"), "urdf", "skeleton.urdf"]
        )
    ])
    robot_description = {"robot_description": robot_description_content}

    # 2. Get the controller configuration from your hardware package
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("rocker_bogie_control"), "config", "ros2_controllers.yaml"]
    )

    # 3. Start the Robot State Publisher to satisfy the Controller Manager
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 4. Start the Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # 5. Spawn the controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # 6. Teleop Test
    test_teleop = Node(
            package="teleop",
            executable="teleop",
            output="screen",
        )

    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        test_teleop,
    ])