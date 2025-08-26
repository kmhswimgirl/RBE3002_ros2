#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.declare_parameter('frame_id', 'map')
        self.path_pub = self.create_publisher(Path, '/my_path', 10)
        self.publish_path()

    def tuple_to_posestamped(self, coordinate):
        frame = self.get_parameter('frame_id').get_parameter_value().string_value
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(coordinate[0])
        pose.pose.position.y = float(coordinate[1])
        pose.pose.orientation.w = 1.0
        return pose

    def publish_path(self):
        frame = self.get_parameter('frame_id').get_parameter_value().string_value
        path_msg = Path()
        path_msg.header.frame_id = frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        path_coords = [(3, 0), (4, 1), (5, 2)]  # Example path
        for coord in path_coords:
            path_msg.poses.append(self.tuple_to_posestamped(coord))

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path {path_coords}. In frame: {frame}')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    

if __name__ == '__main__':
    main()
