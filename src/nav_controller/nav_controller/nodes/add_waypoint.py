#!/usr/bin/env python3

from rclpy.node import Node
from nav_controller.msg import Waypoint

class AddWayPoint(Node):
    def __init__(self):
        super().__init__('add_waypoint')
        self.get_logger().info('Initialising add waypoint node...')
        self.subscriber = self.create_subscription(
            Waypoint,
            '/add_waypoint',
            self.listener_callback,
            10)
        self.get_logger().info('listening to /add_waypoint topic')
        self.subscriber
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Received waypoint: x={msg.x}, y={msg.y}, z={msg.z}')


