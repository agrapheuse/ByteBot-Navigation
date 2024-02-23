#!/usr/bin/env python3

import rclpy
from nav_controller.nodes.show_all_commands import ShowCommands
from nav_controller.nodes.clicked_point_listener import ClickedPointListener

def main():
    rclpy.init()

    clicked_point_listener = ClickedPointListener()
    rclpy.spin(clicked_point_listener)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
