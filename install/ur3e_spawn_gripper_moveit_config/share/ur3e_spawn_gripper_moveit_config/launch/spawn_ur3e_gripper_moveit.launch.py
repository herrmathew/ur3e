from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_path",
            default_value="/home/admin-dev/ur_ws/src/ur_sim/worlds/pick_place_demo.world",
            description="Path to the world file to load",
        )
    )

    # Initialize Arguments
    world_path = LaunchConfiguration("world_path")

    # Get URDF via xacro
    robot_description_content = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_sim"), "/launch/spawn_ur3e_gripper.launch.py"]
        ),
        launch_arguments={
            "world_path": world_path,
        }.items(),
    )

    # Get SRDF via xacro
    robot_description_semantic_content = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur3e_spawn_gripper_moveit_config"),
                        "config",
                        "ur3e.srdf",
                    ]
                )
            ]
        )
    )

    # MoveIt Configuration
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
        ],
    )

    # RViz Configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur3e_spawn_gripper_moveit_config"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"robot_description": robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    for declared_argument in declared_arguments:
        ld.add_action(declared_argument)

    # Add nodes to launch description
    ld.add_action(robot_description_content)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)

    return ld