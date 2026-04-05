#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64MultiArray, Float64

class FieldTest(Node):
    def __init__(self):
        super().__init__('field_test')

        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.max_dist = self.create_publisher(String, '/max_dist', 10)

    def lidar_callback(self, msg:LaserScan): 
        ranges = msg.ranges
        largest_distance = max(ranges)
        direction = ranges.index(largest_distance) * msg.angle_increment + msg.angle_min
        heading = math.degrees(direction)
        string = f'largest distance is {largest_distance}m, at {heading} deg'
        self.max_dist.publish(string)
        

def main(args=None):
    rclpy.init(args=args)
    node = FieldTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
