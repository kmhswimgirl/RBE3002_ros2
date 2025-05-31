#!/usr/bin/env python3

# RBE 3002: Lab #2 (Jazzy Jalisco Version)
import rclpy
import math
import time

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped


class Lab2(Node):

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


    def send_speed(self, linear_speed: float, angular_speed: float):
        msg_cmd_vel = Twist()

        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0

        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        #self.cmd_vel.publish(msg_cmd_vel)

    def drive(self, distance: float, linear_speed: float):

        error = 1000
        tol = 0.05
        init_x = self.px
        init_y = self.py

        while abs(error) > tol:
            dist_traveled = math.sqrt((self.px-init_x)**2 + (self.py - init_y)**2) # euclidean dist. calculation
            self.send_speed(linear_speed,0.0)
            error = distance - dist_traveled


        self.send_speed(0.0,0.0) # stop robot when loop is complete (i.e. distance is below tolerance)

    def rotate(self, target_heading: float, ang_speed: float):
        tolerance = 0.05  # radians
        def normalize_angle(angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle

        while rclpy.ok():
            error = normalize_angle(target_heading - self.pth)
            if abs(error) < tolerance:
                break
            direction = 1.0 if error > 0 else -1.0
            #self.send_speed(0.0, direction * abs(ang_speed))
            rclpy.spin_once(self, timeout_sec=0.01)
        #self.send_speed(0.0, 0.0)

    def go_to(self, msg: PoseStamped):
        if not hasattr(self, 'px') or not hasattr(self, 'py') or not hasattr(self, 'pth'):
            self.get_logger().warn("Odometry not ready, ignoring goal!")
            return

        init_x = self.px
        init_y = self.py

        new_pos_x = msg.pose.position.x
        new_pos_y = msg.pose.position.y

        drive_heading = math.atan2((new_pos_y - init_y), (new_pos_x - init_x))
        angle_to_turn = drive_heading - self.pth

        # Normalize to [-pi, pi]
        while angle_to_turn > math.pi:
            angle_to_turn -= 2 * math.pi
        while angle_to_turn < -math.pi:
            angle_to_turn += 2 * math.pi

        self.rotate(self.pth + angle_to_turn, 0.5)

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
        rclpy.spin(self)

if __name__ == '__main__':
    rclpy.init()
    robot = Lab2(0,0,0)
    robot.run()
    rclpy.get_logger().info('Driving')