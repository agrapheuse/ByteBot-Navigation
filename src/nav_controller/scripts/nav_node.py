#!/usr/bin/env python3

import rclpy
from nav_controller.nodes.nav_points import NavigatorNode
from nav_controller.nodes.show_all_commands import ShowCommands

def main():
    rclpy.init()

    pose_subscriber = NavigatorNode()
    pose_subscriber.get_logger().info('Pose subscriber node is running...')
    rclpy.spin(pose_subscriber)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
