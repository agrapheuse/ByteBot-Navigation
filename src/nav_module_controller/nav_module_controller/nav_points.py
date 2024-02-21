#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import String

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.get_logger().info('Initialising pose subscriber node...')
        self.navigator = TurtleBot4Navigator()

        # Start on dock
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initialising pose')
            self.navigator.dock()

        # Set initial pose
        initial_pose = self.navigator.getPoseStamped([0.001, 0.044], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()

        self.subscription = self.create_subscription(
            String,
            '/pose_listener',
            self.listener_callback,
            10)
        self.get_logger().info('listening to /pose_listener topic')
        self.subscription

    def listener_callback(self, msg):
        goal_location = msg.data

        # Determine goal pose based on user input
        if goal_location == 'undock':
            self.get_logger().info('Undocking...')
            self.navigator.undock()
            return 
        elif goal_location == 'entrance':
            goal_pose = self.navigator.getPoseStamped([-7.196, 3.729], TurtleBot4Directions.SOUTH)  # entrance of the room
        elif goal_location == 'corner':
            goal_pose = self.navigator.getPoseStamped([-6.117, -3.669], TurtleBot4Directions.WEST)  # top left corner of the room
        elif goal_location == 'storage_room':
            goal_pose = self.navigator.getPoseStamped([-4.876, 13.329], TurtleBot4Directions.WEST)  # cisco storage room
        elif goal_location == 'dock':
            goal_pose = self.navigator.getPoseStamped([0.001, 0.044], TurtleBot4Directions.NORTH)
        else:
            self.get_logger.warn("Invalid location entered. Please enter 'entrance', 'corner', or 'storage_room'.")

        # Go to each goal pose
        self.navigator.startToPose(goal_pose)

        if goal_location == 'dock':
            self.navigator.dock()

def main():
    rclpy.init()

    pose_subscriber = NavigatorNode()
    pose_subscriber.get_logger().info('Pose subscriber node is running...')
    rclpy.spin(pose_subscriber)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
