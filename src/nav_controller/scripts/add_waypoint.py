#!/usr/bin/env python3

import rclpy
from nav_controller.nodes.show_all_commands import ShowCommands
from nav_controller.nodes.add_waypoint import AddWaypoint

def main():
    rclpy.init()

    add_waypoint = AddWaypoint()
    rclpy.spin(add_waypoint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
