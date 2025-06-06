#!/usr/bin/env python3

# RBE 3002: Lab #2 (Jazzy Jalisco Version)
import rclpy
import math
import time

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
#from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R

class Lab2 (Node):

    def __init__(self, px:float, py:float, pth:float):
        super().__init__('lab2')

        # self.px = px
        # self.py = py
        # self.pth = pth

        self.px = 0
        self.py = 0
        self.pth = 0
        

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.go_to, 10)
        
        # publishers
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("lab2 node initalized!")


    def send_speed(self, linear_speed: float, angular_speed: float):
        msg_cmd_vel = Twist()

        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0

        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        self.get_logger().info(f"send_speed called with linear: {linear_speed} ({type(linear_speed)}), angular: {angular_speed} ({type(angular_speed)})")

        self.cmd_vel.publish(msg_cmd_vel)

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

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def rotate(self, angle: float, ang_speed: float):
        tolerance = 0.05
        goalAngle = self.normalize_angle(self.pth + angle)
        error = self.normalize_angle(goalAngle - self.pth)
        while abs(error) > tolerance:
            self.get_logger().info(f"Current heading: {self.pth:.3f}, Target: {goalAngle:.3f}, Error: {error:.3f}")
            direction = ang_speed if error > 0 else -ang_speed
            self.send_speed(0.0, direction)
            error = self.normalize_angle(goalAngle - self.pth)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.send_speed(0.0, 0.0)
        self.get_logger().info("ending rotation")

    def go_to(self, msg: PoseStamped):
        
        self.get_logger().info("begin go_to")
        
        #desired position
        new_y = msg.pose.position.y
        new_x = msg.pose.position.x
        init_x = self.px
        init_y = self.py

        self.get_logger().info("go_to: variables set")
        new_ang = math.atan2((new_y - init_y),(new_x - init_x))
        angle_to_turn = new_ang - self.pth
        self.rotate(angle_to_turn, 0.5)

      
        dist =  math.sqrt((new_x - init_x)** 2 + (new_y - init_y)** 2)
        self.drive(dist, 0.1)

        self.get_logger().info("finished go_to")

    def update_odometry(self, msg: Odometry):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = r.as_euler('xyz')
        self.pth = euler[2]

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

if __name__ == '__main__':
    rclpy.init()
    robot = Lab2(0.0,0.0,0.0)
    robot.run()