from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_navigation, 'maps', 'warehouse.yaml']),
        description='Full path to map yaml file to load')


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
        # Node(
        #     package='nav_controller',
        #     executable='add_waypoint.py',
        #     name='add_waypoint'
        # ),
        Node(
            package="nav_controller",
            executable="change_state_node.py",
            name="change_state"
        )
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'turtlebot4_navigation', 'localization.launch.py', 'map:=room505.yaml'],
        #     output='screen',
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'turtlebot4_navigation', 'nav2.launch.py'],
        #     output='screen',
        # )
    ])