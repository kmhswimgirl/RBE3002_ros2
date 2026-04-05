#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TwistStamped, PoseStamped
from rbe3002_interfaces.action import DrivePath

from tf2_geometry_msgs import PoseStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

class Controller(Node):
    def __init__(self, px, py, pth):
        super().__init__('controller')

        self.px = px
        self.py = py
        self.pth = pth

        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

        self.rate = self.create_rate(30) # Hz
        self.cb_group = ReentrantCallbackGroup() 

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry, 10, callback_group=self.cb_group)
        self.rviz_goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.get_path_from_planner, 10, callback_group=self.cb_group)
        self.sub_goal = self.create_subscription(PoseStamped, '/nav_goal', self.get_path_from_planner, 10, callback_group=self.cb_group)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.drive_action = ActionServer(self, DrivePath, '/drive_path',
                                         execute_callback=self.drive_path,
                                         goal_callback=self.drive_path_goal_callback,
                                         cancel_callback=self.drive_path_cancel_callback,
                                         callback_group=self.cg_group)

    
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
    
    def handle_path(self, path:Path):
        '''
        A callback function that handles iterating through a path.
        :param path [Path] The path that the robot will drive.
        '''
        # if no poses recieved log that
        if not path.poses:
            self.get_logger().warn('Received empty path.')
            return

        # if recieved then say so and use go to to interate through the poses
        self.get_logger().info(f'Received path with {len(path.poses)} poses.')
        for i, pose in enumerate(path.poses):
            x_way = round(path.poses[i].pose.position.x, 3)
            y_way = round(path.poses[i].pose.position.y, 3) 
            self.get_logger().info(f'Waypoint ({x_way ,y_way}) {i + 1}/{len(path.poses)}')
            self.go_to(pose)
        self.get_logger().info('Completed path.')
    
    
    async def drive_path(self, goal_handle:Path): 
        "drives to each point in the recieved path through the action"

        self.get_logger().info("DrivePath action initiated")
        path = goal_handle.request.path
        result = DrivePath.Result()

        if not path.poses:
            self.get_logger().warn("DrivePath goal has empty path")
            result.success = False
            result.message = "Empty path"
            goal_handle.abort()
            return result

        for i, pose in enumerate(path.poses):
            # cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info("DrivePath goal canceled by client")
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled"
                return result

            # publish feedback
            feedback = DrivePath.Feedback()
            feedback.current_index = i
            feedback.current_target = pose
            goal_handle.publish_feedback(feedback)

            x_waypoint = round(pose.pose.position.x, 3)
            y_waypoint = round(pose.pose.position.y, 3)
            self.get_logger().info(
                f"DrivePath: waypoint {i+1}/{len(path.poses)} at ({x_waypoint}, {y_waypoint})"
            )

            self.go_to(pose)

        self.get_logger().info("DrivePath goal completed successfully")
        result.success = True
        result.message = "Path completed"
        goal_handle.succeed()
        return result

    def drive_path_goal_callback(self, goal_request: DrivePath.Goal):
        self.get_logger().info(
            f"Received DrivePath goal with {len(goal_request.path.poses)} poses"
        )
        return GoalResponse.ACCEPT

    def drive_path_cancel_callback(self, goal_handle):
        self.get_logger().info("Received DrivePath cancel request")
        return CancelResponse.ACCEPT
    
    
def main(args=None):  # pragma: no cover
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  # pragma: no cover
    main()