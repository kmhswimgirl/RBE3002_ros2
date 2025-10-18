#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped
from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Controller(Node):
    def __init__(self, px, py, pth):
        super().__init__('controller')

        self.px = px
        self.py = py
        self.pth = pth

        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

        self.odom_ready = False

        self.cb_group = ReentrantCallbackGroup() # allow for multiple things at once

        self.rate = self.create_rate(30) # Hz

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry, 10, callback_group=self.cb_group)
        self.get_test_path = self.create_subscription(Path,'/path_generator', self.handle_path, 10, callback_group=self.cb_group)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.get_logger().info("Controller Initialized")

    def update_odometry(self, msg: Odometry): # same as previously, just used a diff library 
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = rotation.as_euler('xyz')
        self.pth = euler[2] # get the yaw value from the returned array
        self.odom_ready = True # set odom data flag to true
    
    def send_speed(self, linear_speed, angular_speed): 
        msg = TwistStamped()
        msg.twist.angular.z = float(angular_speed)
        msg.twist.linear.x = float(linear_speed)

        self.cmd_vel_pub.publish(msg) 

    def drive(self, distance: float, linear_speed: float):
        error = 1000
        tolerance = 0.02 # meters
        initialx = self.px
        initialy = self.py
        
        while abs(error) > tolerance and rclpy.ok(): # while robot is not close enough to goal
        
            distanceTraveled = math.sqrt((self.px - initialx)**2 + (self.py - initialy)**2)
            self.send_speed(linear_speed, 0) 
            error = distance - distanceTraveled
            
            self.rate.sleep() # small delay
            
        self.send_speed(0,0) 

    def rotate(self, angle: float, aspeed: float):
        error = 10000
        tolerance = 0.03 # radians?
        goalAngle  = angle

        # account for 0 and 2pi being the same position etc.
        if goalAngle < 0 and abs(goalAngle) > math.pi:
            goalAngle = 2*math.pi - goalAngle
        elif goalAngle > 0 and goalAngle > math.pi:
            goalAngle = goalAngle - 2*math.pi

        while (abs(error) > tolerance) and rclpy.ok(): # while robot is not close enough to goal
            if(goalAngle < self.pth):
                self.send_speed(0, -aspeed)
            else:
                self.send_speed(0, aspeed)

            error = self.pth - goalAngle 
            self.rate.sleep()
                    
        self.send_speed(0,0) 

    def go_to(self, msg: PoseStamped):
        goal_pos_x = msg.pose.position.x
        goal_pos_y = msg.pose.position.y

        initial_x = self.px
        initial_y = self.py

        newAngle = math.atan2((goal_pos_y - initial_y), (goal_pos_x - initial_x))

        self.rotate(newAngle, 0.5)

        distance = math.sqrt((goal_pos_x - initial_x)**2 + (goal_pos_y - initial_y)**2) 
        self.drive(distance, 0.1) 
    
    def handle_path(self, path:Path):
        poses = path.poses
        for pose in poses:
            self.get_logger().info(f'driving to {pose}')
            self.go_to(pose)
        
        self.get_logger().info("done driving path")
    
def main(args=None): # i think i got this right? idk
    rclpy.init(args=args)
    node = Controller(0,0,0)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()