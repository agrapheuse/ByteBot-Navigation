#!/usr/bin/env python3

from rclpy.node import Node
from nav_module_interfaces.msg import TargetCoordinates
import json
import os
from ament_index_python.packages import get_package_share_directory

class AddWayPoint(Node):
    def __init__(self):
        super().__init__('add_waypoint')
        self.get_logger().info('Initialising add waypoint node...')
        self.subscriber = self.create_subscription(
            TargetCoordinates,
            '/add_waypoint',
            self.listener_callback,
            10)
        self.get_logger().info('listening to /add_waypoint topic')
        self.package_share_directory = get_package_share_directory('nav_controller')
        self.json_file_path = os.path.join(self.package_share_directory, 'resources', 'commands.json')
        self.subscriber
        self.goals = self.retrieve_commands()
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Received waypoint: name={msg.name}, position=[{msg.coordinates.x}, {msg.coordinates.y}]')
        self.get_logger().info(f'Adding waypoint to goals list...')
        self.goals[msg.name] = {'position': [msg.coordinates.x, msg.coordinates.y], 'orientation': 90}
        self.save_commands()

    def retrieve_commands(self):
        with open(self.json_file_path, 'r') as file:
            return json.load(file)

    def save_commands(self):
        with open(self.json_file_path, 'w') as file:
            json.dump(self.goals, file, indent=4)
            self.get_logger().info('Commands updated and saved to commands.json')

