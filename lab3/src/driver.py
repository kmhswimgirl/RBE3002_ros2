#!/usr/bin/env python3

# RBE 3002: Lab #2 (Jazzy Jalisco Version) (that actually works!)
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from scipy.spatial.transform import Rotation as R
from nav_msgs.srv import GetPlan, GetMap, GetPlanResponse

class Driver(Node):
    # brings back memories from 2002
    ROBOT_IDLE = 0
    ROBOT_ROTATE = 1
    ROBOT_DRIVE = 2
    # ROBOT_DONE = 3

    def __init__(self):
        super().__init__('driver')
        # have to init variables to avoid attribute error

        # constructor
        self.px = None
        self.py = None
        self.pth = None
        self.odom_ready = False

        # goal location
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        self.state = self.ROBOT_IDLE # initial state

        # path stuff
        self.path_to_goal = None

        # subscribers and publishers. i think this syntax is correct. ros2 can be weird compared to Noetic lol
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.go_to, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.state_machine) # self.state_machine is called every 0.05 seconds

        # client service call
        self.path_client = self.create_client(GetPlan, '/path_planner/plan_path')

        self.get_logger().info("driver node initialized") # confirmation that i did not screw anything above up too badly

    def update_odometry(self, msg: Odometry): # same as previously, just used a diff library 
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = rotation.as_euler('xyz')
        self.pth = euler[2] # get the yaw value from the returned array
        self.odom_ready = True # set odom data flag to true
    
    def get_path_from_planner(self, start:PoseStamped, goal:PoseStamped, tol=0.2):
        path_req = GetPlan.Request()
        path_req.start = start
        path_req.goal = goal
        path_req.tolerance = tol

        while not self.path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for path_planner to generate path!")

    def normalize_angle(self, angle): # added to make code more modular. also before i think i did this wrong...
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def send_speed(self, linear_speed, angular_speed): # publish speed to /cmd_vel topic (MAKE SURE TO CHANGE GZ_BRIDGE!!!!!!!!)
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(msg) # publish speed to /cmd_vel topic

    def go_to(self, msg: PoseStamped): # callback for placing a goal location in rviz
        if not self.odom_ready:
            self.get_logger().warn("no /odom data")
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        dx = self.goal_x - self.px # x delta
        dy = self.goal_y - self.py # y delta
        self.goal_theta = math.atan2(dy, dx) # angle goal is at

        # initial position of the robot
        self.start_x = self.px
        self.start_y = self.py

        # first state in turn, drive, turn is ofc turn
        self.state = self.ROBOT_ROTATE
        self.get_logger().info(f"goal: ({self.goal_x:.2f}, {self.goal_y:.2f}), angle: {self.goal_theta:.2f} ")

    def state_machine(self): # controls robot motion based on curent state. should be non-blocking and allow odom to update
        if not self.odom_ready: # checks if /odom is ready
            return

        if self.state == self.ROBOT_IDLE: # if robot state is idle, aka not doing anything
            self.send_speed(0.0, 0.0)
            return

        elif self.state == self.ROBOT_ROTATE: # same as rotate()
            angle_error = self.normalize_angle(self.goal_theta - self.pth)
            tol = 0.05
            ang_speed = 0.5
            if abs(angle_error) > tol:
                direction = ang_speed if angle_error > 0 else -ang_speed # change direction based on goal pos
                self.send_speed(0.0, direction)
            else:
                self.send_speed(0.0, 0.0)
                self.drive_dist = math.sqrt((self.goal_x - self.px) ** 2 + (self.goal_y - self.py) ** 2) # calc distance needed to travel
                self.start_x = self.px
                self.start_y = self.py
                self.state = self.ROBOT_DRIVE # set to next state
                self.get_logger().info("done spinning")

        elif self.state == self.ROBOT_DRIVE: # same as drive()
            dist_traveled = math.sqrt((self.px - self.start_x) ** 2 + (self.py - self.start_y) ** 2)
            dist_remaining = self.drive_dist - dist_traveled
            tol = 0.05
            linear_speed = 0.1
            if dist_remaining > tol:
                self.send_speed(linear_speed, 0.0)
            else:
                self.send_speed(0.0, 0.0)
                self.state = self.ROBOT_IDLE # stops robot
                self.get_logger().info("done")

def main(args=None): # i think i got this right? idk
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()