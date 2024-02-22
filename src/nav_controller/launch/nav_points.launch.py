from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_controller',
            node_executable='nav_points',
            name='navigation_controller'
        )
    ])