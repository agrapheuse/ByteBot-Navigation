#!/usr/bin/env python3

from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import String
import json
import os
from ament_index_python.packages import get_package_share_directory

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
        initial_pose = self.navigator.getPoseStamped([0.104, 0.063], TurtleBot4Directions.NORTH_WEST) # cisco room
        # initial_pose = self.navigator.getPoseStamped([0.017, -0.019], TurtleBot4Directions.NORTH) # 505 room
        self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('Retrieving commands...')
        self.retrieve_commands()
        self.get_logger().info('Retrieved goals:')
        for i in self.goals:
            self.get_logger().info(i)
        self.get_logger().info('Commands retrieved')

        self.subscription = self.create_subscription(
            String,
            '/pose_listener',
            self.listener_callback,
            10)
        self.get_logger().info('listening to /pose_listener topic')
        self.subscription

    def retrieve_commands(self):
        package_share_directory = get_package_share_directory('nav_controller')
        json_file_path = os.path.join(package_share_directory, 'resources', 'commands.json')

        with open(json_file_path, 'r') as file:
            self.goals = json.load(file)

    def listener_callback(self, msg):
        self.retrieve_commands()
        goal_location = msg.data

        # Determine goal pose based on user input
        if goal_location == 'undock':
            self.get_logger().info('Undocking...')
            self.navigator.undock()
            return
        elif goal_location == 'dock':
            position = self.goals[goal_location]['position']
            goal_pose = self.navigator.getPoseStamped(position, TurtleBot4Directions(self.goals[goal_location]['orientation']))
            self.get_logger().info('Docking...')
            self.navigator.dock()
            return
        elif goal_location in self.goals:
            position = self.goals[goal_location]['position']
            goal_pose = self.navigator.getPoseStamped(position, TurtleBot4Directions(self.goals[goal_location]['orientation']))
        else:
            self.get_logger.warn("Invalid location entered. Please enter a valid command from the follosing list: ")
            for i in self.goals:
                self.get_logger().info(i)
            return

        self.navigator.startToPose(goal_pose)

