from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_controller',
            executable='nav_node.py',
            name='navigation_controller'
        ),
        Node(
            package='nav_controller',
            executable='show_commands.py',
            name='show_all_commands'
        )
    ])