#!/usr/bin/env python3

# RBE 3002: Lab #2 (Jazzy Jalisco Version)
import rclpy
import math
import time
import tf_transformations

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from tf_transformations import euler_from_quaternion

class Lab2 (Node):

    def __init__(self, px, py, pth):
        # Constructor
        self.px = px
        self.py = py
        self.pth = pth

        # init node 'lab2'
        super().__init__('lab2')

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.go_to, 10)
        
        # publishers
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("lab2 Initalized!")

        self.drive(5.0, 0.1)


    def send_speed(self, linear_speed: float, angular_speed: float):
        self.get_logger().info("starting send_speed")

        msg_cmd_vel = Twist()

        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0

        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        self.cmd_vel.publish(msg_cmd_vel)
        self.get_logger().info("ending send_speed")

    def drive(self, distance: float, linear_speed: float):
        self.get_logger().info("starting drive")

        error = 1000
        tol = 0.05
        init_x = self.px
        init_y = self.py

        while abs(error) > tol:
            dist_traveled = math.sqrt((self.px-init_x)**2 + (self.py - init_y)**2) # euclidean dist. calculation
            self.send_speed(linear_speed,0.0)
            error = distance - dist_traveled
            rclpy.spin_once(self, timeout_sec=0.01)

        self.send_speed(0.0, 0.0) # stop robot when loop is complete (i.e. distance is below tolerance)
        self.get_logger().info("ending drive")

    def rotate(self, target_heading: float, ang_speed: float):
        error = 1000
        tolerance = 0.2
        target_heading = self.normalize_angle((self.pth + target_heading))
    
        while abs(error) > tolerance:
            current_heading = self.pth
            self.get_logger().info(f"Current heading: {self.pth}")
            
            self.send_speed(0.0, ang_speed)
            rclpy.spin_once(self, timeout_sec=0.05)
            error = self.normalize_angle((target_heading - current_heading))

        self.send_speed(0.0, 0.0)
        self.get_logger().info("ending rotation")

    def go_to(self, msg: PoseStamped):
        
        self.get_logger().info("begin go_to")

        #current robor position
        init_th = self.pth
        
        #desired position
        new_y = msg.pose.position.y
        new_x = msg.pose.position.x
        init_x = self.px
        init_y = self.py

        self.get_logger().info("go_to: variables set")
        new_ang = math.atan2((new_y - init_y),(new_x - init_x))
        angle_to_turn = new_ang - self.pth
        self.rotate(self.pth + angle_to_turn, 0.5)

        time.sleep(0.05)
        dist =  math.sqrt((new_x - init_x)** 2 + (new_y - init_y)** 2)
        self.drive(dist, 0.1)

        self.get_logger().info("finished go_to")


    def update_odometry(self, msg: Odometry):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        (r, p, y) = tf_transformations.euler_from_quaternion([quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w])
        self.pth = y

    def smooth_drive(self, distance: float, linear_speed: float):
        pass

    def run(self):
        rclpy.spin(self)

if __name__ == '__main__':
    rclpy.init()
    robot = Lab2(0,0,0)
    robot.run()