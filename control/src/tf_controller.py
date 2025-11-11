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
from tf2_geometry_msgs import PoseStamped
import tf2_geometry_msgs

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

class Controller(Node):
    def __init__(self, px, py, pth):
        super().__init__('controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('angular_speed', 0.5),
                ('linear_speed', 0.1),
                ('angular_tolerance', 0.02), 
                ('linear_tolerance', 0.03),
                ('path_tolerance', 0.03),
                ('poses_before_goal', 1),
                ('delay_rate', 30)
            ])

        self.px = px
        self.py = py
        self.pth = pth

        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

        # speeds + tolerances
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        self.linear_tolerance = self.get_parameter('linear_tolerance').value
        self.delay_rate = self.get_parameter('delay_rate').value

        self.path_to_goal = None
        self.current_path_index = 0

        self.path_tolerance = self.get_parameter('path_tolerance').value
        self.stop_before_goal = self.get_parameter('poses_before_goal').value

        self.odom_ready = False

        self.cb_group = ReentrantCallbackGroup() # allow for multiple things at once

        self.rate = self.create_rate(self.delay_rate) # Hz

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry, 10, callback_group=self.cb_group)
        self.rviz_goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.get_path_from_planner, 10, callback_group=self.cb_group)
        self.sub_goal = self.create_subscription(PoseStamped, '/nav_goal', self.get_path_from_planner, 10, callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.path_client = self.create_client(GetPlan, '/path_planner/plan_path')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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

    def normalize_angle(self, angle): # yoinked from a diff version
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def drive(self, distance: float, linear_speed: float):
        '''
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        '''
        error = 1000
        tolerance = self.linear_tolerance # meters
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
        tolerance = self.angular_tolerance
        goalAngle = angle

        while rclpy.ok():
            if not self.odom_ready:
                self.get_logger().warn('rotate: no odom, stopping')
                self.send_speed(0.0, 0.0)
                return
            error = self.normalize_angle(goalAngle - self.pth)
            if abs(error) <= tolerance:
                break
            direction = aspeed if error > 0 else -aspeed
            self.send_speed(0.0, direction)
            self.rate.sleep()

        self.send_speed(0,0) 

    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """

        pose_in_odom = self._tf_buffer.transform(msg,'odom', rclpy.duration.Duration(seconds=1))
   
        goal_pos_x = pose_in_odom.pose.position.x
        goal_pos_y = pose_in_odom.pose.position.y

        initial_x = self.px
        initial_y = self.py

        newAngle = math.atan2((goal_pos_y - initial_y), (goal_pos_x - initial_x))

        self.rotate(newAngle, self.angular_speed)

        distance = math.sqrt((goal_pos_x - initial_x)**2 + (goal_pos_y - initial_y)**2) 
        self.drive(distance, self.linear_speed) 
        
        self.get_next_point()
    
    def get_path_from_planner(self, msg: PoseStamped):
        if not self.odom_ready:
            self.get_logger().warn("No odometry data yet!")
            return
        
        self.get_logger().info('service callback called!')

        # start pose from current odom
        start = PoseStamped()
        start.header.frame_id = "odom"  # watch the frames
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
        path_req.tolerance = self.path_tolerance

        while not self.path_client.wait_for_service(timeout_sec=1.0): # wait for service
            self.get_logger().info("waiting for path")
            self.send_speed(0.0, 0.0)

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
                self.send_speed(0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f"failed to get path {e}")
            self.send_speed(0.0, 0.0)

    def get_next_point(self): # iterate to the next pose in the path that was generated from path_planner
        if self.path_to_goal and self.current_path_index < len(self.path_to_goal.poses) - 1: # stop at pose before goal
            self.current_path_index += 1
            next_pose = self.path_to_goal.poses[self.current_path_index]
            self.go_to(next_pose)
            self.get_logger().info(f"Moving to waypoint {self.current_path_index}/{len(self.path_to_goal.poses)- self.stop_before_goal}")
        else:
            self.get_logger().info("Reached final destination!")
            self.send_speed(0.0, 0.0)
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