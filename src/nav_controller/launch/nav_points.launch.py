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
        ),
        Node(
            package='nav_controller',
            executable='clicked_point_listener.py',
            name='clicked_point_listener'
        ),
        Node(
            package='nav_controller',
            executable='add_waypoint.py',
            name='add_waypoint'
        ),
    ])