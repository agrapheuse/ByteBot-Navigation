#!/usr/bin/env python3

from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from ament_index_python.packages import get_package_share_directory

class ShowCommands(Node):
    def __init__(self):
        super().__init__('show_commands')
        self.get_logger().info('Initialising show commands node...')
        self.subscription = self.create_subscription(
            String,
            '/pose_listener',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        list_of_commands = self.retrieve_commands()
        if msg.data == 'list':
            self.get_logger().info('List of commands:')
            self.get_logger().info('-----------------')
            self.get_logger().info('sleep')
            self.get_logger().info('wake up')
            self.get_logger().info('stop')
            self.get_logger().info('patrol')

            for i in list_of_commands:
                self.get_logger().info(i)

    def retrieve_commands(self):
        package_share_directory = get_package_share_directory('nav_controller')
        json_file_path = os.path.join(package_share_directory, 'resources', 'commands.json')

        with open(json_file_path, 'r') as file:
            goals = json.load(file)
        return goals
