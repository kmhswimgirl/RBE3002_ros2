#!/usr/bin/env python3

import rclpy
import math

from rclpy.node import Node
from typing import List, Tuple
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry

class TestPath(Node):
    def __init__(self):
        super().__init__('test_path')
        self.loginfo = self.get_logger().info

        # location data
        self.px = None
        self.py = None
        self.pth = None

        # subscribe to TurtleBot location for initial position
        self.turtlebot_location = self.create_subscription(Odometry, '/odom', self.set_init_pose)

    def set_init_pose(self, msg: Odometry):


    def generate_path(self):
        '''
        Psuedo code for this one
        '''