import rclpy
import math
from typing import List, Tuple
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from map_methods import MapMethods
from rclpy.executors import MultiThreadedExecutor
from exploration_interfaces import FrontierArray

class Scoring(Node):
    def __init__(self):
        super().__init__('scoring')

        self.map = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.frontiers = self.create_subscription(FrontierArray, '/frontiers', self.main_scoring, 10)

    def map_callback(self, map: OccupancyGrid):
        self.mapdata = map

    def classifying_frontier_shapes():
        # get x range
        # get y range

        # if delta x > delta y, x dominant line
            # else y dominant line

        # "squareness", navgoal viable/non-viable
        pass

    def location_scoring(self, drive_goals: List[Tuple[int, int]]):
        pass

    def proximity_score(self):
        # calc distance forom one centroid to all the others
        # best choice = lowest score
        pass


    def main_scoring(self, level:int):
        # simple:
            # proximity score (distance to other frontier centroids)
            # frontier size
            # displacement from location
        # intermediate
            # all of simple
            # 
        # all


        pass
