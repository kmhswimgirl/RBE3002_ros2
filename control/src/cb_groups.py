#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
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

        cb_group = ReentrantCallbackGroup() # allow for multiple things at once

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry, 10, callback_group=cb_group)
        self.sub_goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.go_to, 10, callback_group=cb_group)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.get_logger().info("driver node initialized")

    def update_odometry(self, msg: Odometry): # same as previously, just used a diff library 
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = rotation.as_euler('xyz')
        self.pth = euler[2] # get the yaw value from the returned array
        self.odom_ready = True # set odom data flag to true

    def normalize_angle(self, angle): # added to make code more modular. also before i think i did this wrong...
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def send_speed(self, linear_speed, angular_speed): 
        msg = TwistStamped()
        msg.twist.angular.z = float(angular_speed)
        msg.twist.linear.x = float(linear_speed)
        self.cmd_vel_pub.publish(msg) # publish speed to /cmd_vel topic

    def drive(self, distance: float, linear_speed: float):  # from 3002 D25
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        error = 1000
        tolerance = 0.05 # Distance tolerance of 0.05 meters
        initialx = self.px
        initialy = self.py
        
        # Run while the distance between the robot's initial position and current position is less than the
        # specified distance (within the tolerance)
        while abs(error) > tolerance:
        
            distanceTraveled = math.sqrt((self.px - initialx)**2 + (self.py - initialy)**2) # Use distance formula to calculate traveled distance
            self.send_speed(linear_speed, 0) # Set linear speed
            error = distance - distanceTraveled # Calculate error between current position and initial position
            
        self.send_speed(0,0) # Stop robot when the while loop is exited, meaning the robot got to the position

    def rotate(self, angle: float, aspeed: float): # from 3002 D25
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        error = 10000
        tolerance = 0.05 # Distance tolerance of 0.1 radians
        goalAngle  = angle

        # Account for when the angle is less than -pi or greater than pi
        # Map it to angles between -pi and pi - ex. if angle given is 3pi/2, make robot go to -pi/2 instead because they are the same position
        if goalAngle < 0 and abs(goalAngle) > math.pi:
            goalAngle = 2*math.pi - goalAngle
        elif goalAngle > 0 and goalAngle > math.pi:
            goalAngle = goalAngle - 2*math.pi

        # Run while the absolute value of the error is greater than the tolerance
        while (abs(error) > tolerance):
        
            # If angle is negative make angular speed negative to go clockwise and if positive, keep angular speed positive to go counterclockwise
            if(goalAngle < self.pth):
                self.send_speed(0, -aspeed)
            else:
                self.send_speed(0, aspeed)

            error = self.pth - goalAngle # Calculate error between current angle and initial angle
            #print(error)
            #print(self.pth)
                    
        self.send_speed(0,0) # Stop robot when the while loop is exited, meaning the robot got to the angle

    def go_to(self, msg: PoseStamped): # from 3002 D25
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        # quat_orig = msg.pose.orientation
        # (roll, pitch, yaw) = euler_from_quaternion([quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w])

        initialth = self.pth # Robot's current angle
        newPositionx = msg.pose.position.x
        newPositiony = msg.pose.position.y

        initialx = self.px # Robot's current x position
        initialy = self.py # Robot's current y position

        newAngle = math.atan2((newPositiony - initialy), (newPositionx - initialx))

        # Calculate the rotation needed to turn the robot to the new angle
      #  differenceAngle = initialth - newAngle

        self.rotate(newAngle, 0.5) # Call rotate function and set speed to 0.5 rad/sec

        # Calculate distance from current position to new position
        distance = math.sqrt((newPositionx - initialx)**2 + (newPositiony - initialy)**2) # Use distance formula to calculate distance needed to travel
        self.drive(distance, 0.1) # Call drive function and set speed to 0.1 m/s

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