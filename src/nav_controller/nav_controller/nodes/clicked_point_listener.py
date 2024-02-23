#!/usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class ClickedPointListener(Node):
    def __init__(self):
        super().__init__('clicked_point_listener')
        self.get_logger().info('Initialising clicked point listener node...')
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.listener_callback,
            10)
        self.get_logger().info('listening to /clicked_point topic')
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'Received point: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}')