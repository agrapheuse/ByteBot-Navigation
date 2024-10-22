#!/usr/bin/env python3

import rclpy
from nav_controller.nodes.show_all_commands import ShowCommands
from nav_controller.nodes.clicked_point_listener import ClickedPointListener

def main():
    rclpy.init()

    command_lister = ShowCommands()
    rclpy.spin(command_lister)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
