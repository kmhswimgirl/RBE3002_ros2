#!/usr/bin/env python3

import rospy # type: ignore
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from driving import Driving

# Subscribe to 2D nav goal (goal) and 2D pose estimate (start)
# Listens to these topics from Rviz, and once gotten, set the start and end to be the start and end fields in a GetPlan message
# Send the GetPlan message to the service to request the path to get from start to goal
# Service will calculate the path using the path_plan function in the path_planner.py file, which it will send back here and go_to function will be used to get robot to drive


class Client:
    
    def __init__(self, initialPose):
        """
        Class constructor
        """
        self.initialPose = initialPose
        ## Initialize node and call it "path_planner_client"
        rospy.init_node("path_planner_client")

        # Subscribe to 2D pose estimate (start) and 2D nav goal (goal) from Rviz and call the appropiate functions
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.handle_path_request)
        self.driving = rospy.Publisher('/driving', PoseStamped, queue_size=1000)


    def initial_pose(self, msg: PoseWithCovarianceStamped):
        # Turn into a PoseStamped message type because that is what the service function takes
        header = msg.header
        pose = msg.pose.pose

        # Set the initial pose field to be the pose sent by the 2D pose estimate
        self.initialPose = PoseStamped()
        self.initialPose.header = header
        self.initialPose.pose = pose


    def handle_path_request(self, msg: PoseStamped):
        start_end_points = GetPlan()
        start_end_points.start = self.initial_pose
        start_end_points.goal = msg

        drive_to_poses = []

        rospy.loginfo("Requesting the path")

        # Try to connect to service
        rospy.wait_for_service('/plan_path')

        # Try to send a request to the service for a path to follow
        try:
            getPlanService = rospy.ServiceProxy('/plan_path', GetPlan)
            path_request = GetPlanRequest()

            # Create a path request using the start and goal positions
            path_request.start = self.initialPose
            path_request.goal = msg

            # Send request to service
            path_response = getPlanService(path_request)

            rospy.loginfo("Got path")

            # Get response from service and store it
            drive_to_poses = path_response.plan.poses
            rospy.loginfo(drive_to_poses)

        except:
            rospy.loginfo("Couldn't get path")

        # Loop through all poses sent from service and publish a driving message to the driving topic so the robot will drive to each pose
        for pose_stamped in drive_to_poses:
            driving_msg = PoseStamped()
            driving_msg.header = pose_stamped.header
            driving_msg.pose = pose_stamped.pose
            self.driving.publish(driving_msg)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    Client(0).run()