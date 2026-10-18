from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_ur_sim = FindPackageShare('ur_sim')
    
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ur_sim, 'launch', 'spawn_ur3e_gripper.launch.py'])
        ),
        launch_arguments={
            'world_path': PathJoinSubstitution([pkg_ur_sim, 'worlds', 'pick_place_demo.world']),
            'robot_name': 'ur3e',
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.6',  # Adjusted to match table height
        }.items()
    )

    # Add proximity monitor node
    proximity_monitor = Node(
        package='ur_sim',
        executable='proximity_monitor',
        name='proximity_monitor',
        output='screen',
        parameters=[{
            'warning_distance': 2.0,
            'stop_distance': 1.5,
            'update_rate': 10.0
        }]
    )

    ld = LaunchDescription()
    ld.add_action(spawn_robot)
    ld.add_action(proximity_monitor)
    
    return ld