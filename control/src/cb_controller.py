#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
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

        self.rate = self.create_rate(30) # Hz

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry, 10, callback_group=cb_group)
        self.sub_goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.get_path_from_planner, 10, callback_group=cb_group)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.path_client = self.create_client(GetPlan, '/path_planner/plan_path')

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
        '''
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        '''
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
        '''
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        '''
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
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
   
        goal_pos_x = msg.pose.position.x
        goal_pos_y = msg.pose.position.y

        initial_x = self.px
        initial_y = self.py

        newAngle = math.atan2((goal_pos_y - initial_y), (goal_pos_x - initial_x))

        self.rotate(newAngle, 0.5)

        distance = math.sqrt((goal_pos_x - initial_x)**2 + (goal_pos_y - initial_y)**2) 
        self.drive(distance, 0.1) 
        
        self.get_next_point();
    
    def get_path_from_planner(self, msg: PoseStamped):
        if not self.odom_ready:
            self.get_logger().warn("No odometry data yet!")
            return

        # start pose from current odom
        start = PoseStamped()
        start.header.frame_id = "map"  # Use map frame instead of odom
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.px
        start.pose.position.y = self.py
        start.pose.position.z = 0.0

        # Set orientation from odom
        quat = R.from_euler('z', self.pth).as_quat()
        start.pose.orientation.x = quat[0]
        start.pose.orientation.y = quat[1]
        start.pose.orientation.z = quat[2]
        start.pose.orientation.w = quat[3]

        goal = msg  # The goal from RViz

        path_req = GetPlan.Request()
        path_req.start = start
        path_req.goal = goal
        path_req.tolerance = 0.03

        while not self.path_client.wait_for_service(timeout_sec=1.0): # wait for service
            self.get_logger().info("waiting for path")

        planned_path_future = self.path_client.call_async(path_req)
        planned_path_future.add_done_callback(self.handle_path_response)
        self.get_logger().info("requested path.")

    def handle_path_response(self, path_response):
        try:
            response = path_response.result() # thank you robotics stack exchange for this
            if response and response.plan and len(response.plan.poses) > 0:
                self.path_to_goal = response.plan
                self.current_path_index = 1 if len(response.plan.poses) > 1 else 0
                if len(response.plan.poses) > self.current_path_index:
                    self.go_to(response.plan.poses[self.current_path_index])
                self.get_logger().info(f"Received path with {len(self.path_to_goal.poses)} poses")
            else:
                self.get_logger().warn("path empty")
        except Exception as e:
            self.get_logger().error(f"failed to get path")

    def get_next_point(self):  # iterate to the next pose in the path that was generated from path_planner
        if self.path_to_goal and self.current_path_index < len(self.path_to_goal.poses) - 1:
            self.current_path_index += 1
            next_pose = self.path_to_goal.poses[self.current_path_index]
            self.go_to(next_pose)
            self.get_logger().info(f"Moving to waypoint {self.current_path_index}/{len(self.path_to_goal.poses)-1}")
        else:
            self.get_logger().info("Reached final destination!")
            self.path_to_goal = None
            self.current_path_index = 0
    
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