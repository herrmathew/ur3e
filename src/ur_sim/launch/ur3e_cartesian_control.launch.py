from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('ur_sim'),
        'config',
        'ur3e_admittance_controller.yaml'
    ])

    admittance_controller_node = Node(
        package='ur_sim',
        executable='admittance_controller',
        name='admittance_controller',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('wrench', '/wrench'),
            ('joint_states', '/joint_states'),
        ]
    )

    return LaunchDescription([
        admittance_controller_node
    ])
