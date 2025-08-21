#!/usr/bin/env python3

import rclpy
import math

from rclpy.node import Node
from typing import List, Tuple
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

class TestPath(Node):
    def __init__(self):
        super().__init__('test_path')
        self.loginfo = self.get_logger().info

        # subscribe to TurtleBot location for initial position
        self.turtlebot_location = self.create_subscription(PoseStamped, '/init_pose', self.set_init_pose)

        # publish the finished path to the topic test path
        self.turtlebot_path = self.create_publisher(Path, '/test_path', 10)

    def set_init_pose(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        return 
        
    def generate_path(self):
        '''
        Psuedo code for this one:

        init position comes from the def set_init_pose()
        point #1 = Pose + 0.2m forwards
        point #2 = point 1 + 
        point #3 = 
        '''