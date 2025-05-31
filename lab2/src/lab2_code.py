#!/usr/bin/env python3

# RBE 3002: Lab #2 (Jazzy Jalisco Version)
import rclpy
import math
import time

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped


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
        self.get_logger().info("starting rotation")
        tol= 0.05  # radians
        goal_th = target_heading
        ang_err = 1000

        if goal_th < 0 and abs(goal_th) > math.pi:
            goal_th = 2*math.pi - goal_th

        elif goal_th > 0 and goal_th > math.pi:
            goal_th = goal_th - 2*math.pi

        while(abs(ang_err) > tol):
            if(goal_th < self.pth):
                
                self.send_speed(0.0, -ang_speed)
            else:
                self.send_speed(0.0, ang_speed)
            ang_err = abs(self.pth - goal_th)
            rclpy.spin_once(self, timeout_sec=0.01)

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

        #use some trig to get the new theta part of the pose (I <3 trig)
        new_ang = math.atan2((new_y - init_y),(new_x - init_x))
        self.rotate(new_ang, 0.5)

        self.get_logger().info("begin go_to")
        time.sleep(0.05)
        #another use of pythagorean theorem to calc distance (i think its called something Euclidian???)
        # basically a control c, control v from drive() method && change a var name or two
        dist =  math.sqrt((new_x - init_x)** 2 + (new_y - init_y)** 2)
        self.drive(dist, 0.1)

        self.get_logger().info("finished go_to")


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