#!/usr/bin/env python3

# RBE 3002: Lab #2 (Jazzy Jalisco Version)
import rclpy
import math
import time

from rclpy import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped


class Lab2(Node):

    def __init__(self, px, py, pth):
        # Constructor
        self.px = px
        self.py = py
        self.pth = pth

        # init node 'lab2'
        super().__init__('lab2')

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry)
        self.sub_goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.go_to)
        
        # publishers
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        time.sleep(0.01)
        self.get_logger().info("lab2 Initalized!")
        time.sleep(0.01)


        
    def send_speed(self, linear_speed: float, angular_speed: float):
        msg_cmd_vel = Twist()

        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0

        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        self.cmd_vel.publish(msg_cmd_vel)

    def drive(self, distance: float, linear_speed: float):

        error = 1000
        tol = 0.05
        init_x = self.px
        init_y = self.py

        while abs(error) > tol:
            dist_traveled = math.sqrt((self.px-init_x)**2 + (self.py - init_y)**2) # euclidean dist. calculation
            self.send_speed(linear_speed,0)
            error = distance - dist_traveled

            time.sleep(0.05)

        self.send_speed(0,0) # stop robot when loop is complete (i.e. distance is below tolerance)

    def rotate(self, angle: float, ang_speed: float):
        error = 1000
        tol = 0.05
        goal_angle = angle

        if goal_angle < 0 and abs(goal_angle) > math.pi:
            goal_angle = 2*math.pi - goal_angle
        elif goal_angle > 0 and goal_angle > math.pi:
            goal_angle = goal_angle - 2*math.pi
        
        while (abs(error) > tol):

            if(goal_angle < self.pth):
                self.send_speed(0, -ang_speed)
            else:
                self.send_speed(0, ang_speed)
            
            error = self.pth - goal_angle

            time.sleep(0.05)

        self.send_speed(0,0)

    def go_to(self, msg: PoseStamped):
        init_x = self.px
        init_y = self.py

        new_pos_x = msg.pose.position.x
        new_pos_y = msg.pose.position.y

        drive_heading = math.atan2((new_pos_y - init_y), (new_pos_x - init_x))
        self.rotate(drive_heading, 0.5)
        
        time.sleep(0.05)

        drive_dist = self.euclidean_dist(init_x, init_y, new_pos_x, new_pos_y)
        self.drive(drive_dist, 0.1)


    def update_odometry(self, msg: Odometry):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        (r, p, y) = self.euler_from_quaternion(quat_orig)
        self.pth = y

    def smooth_drive(self, distance: float, linear_speed: float):
        pass


    # does not exist in tf2 library to my knowledge
    def euler_from_quaternion(self, quaternion):

        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sin_r = 2 * (w * x + y * z)
        cos_r = 1 - 2 * (x**2 + y**2)
        roll = math.atan2(sin_r, cos_r)

        sin_p = 2 * (w * y - z * x)
        pitch = math.asin(sin_p)

        sin_y = 2 * (w * z + x * y)
        cos_y = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(sin_y, cos_y)

        return roll, pitch, yaw

    def euclidean_dist(self, x1: float, y1: float, x2: float, y2: float):
        output = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        return output

    def run(self):
        rclpy.spin()

if __name__ == '__main__':
    robot = Lab2(0,0,0)
    robot.run()
    rclpy.get_logger().info('Driving')