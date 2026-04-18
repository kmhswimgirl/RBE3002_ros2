#!/usr/bin/env python3
from __future__ import annotations
from typing import List, Tuple
import numpy.typing as npt

from rclpy.node import Node

import numpy as np
import math
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from  nav_msgs.msg import OccupancyGrid, Path, MapMetaData, GridCells
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.srv import GetPlan
import cv2 as cv

# from priority_queue import PriorityQueue 

import tf2_geometry_msgs

class GraphNode():
    def __init__(self,pose,parent,cost):
        self.pose = pose
        self.parent = parent
        self.cost = cost
    
    def __eq__(self,other):
        return self.pose[0] == other.pose[0] and \
               self.pose[1] == other.pose[1]
    
    def __lt__(self,other):

        return id(self) < id(other)

class PathPlanner(Node):
    def __init__(self):

        super().__init__('path_planner')

        qos = QoSProfile(depth=100)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        qos_cp = QoSProfile()
        qos.durability = DurabilityPolicy.VOLATILE

        # map topics
        self.create_subscription(OccupancyGrid, '/map', self.handle_map, qos)
        self.safe_map = self.create_publisher(OccupancyGrid, '/map/safe', qos)

        # clicked point
        self.create_subscription(Point, '/clicked_point', self.clicked_point_callback, qos_cp)
    
        # visualization publishers
        self.expanded_cells = self.create_publisher(GridCells, "/path_planner/expanded_cells", qos) 
        self.frontier_cells = self.create_publisher(GridCells, "/path_planner/frontier", qos)
        self.c_space = self.create_publisher(GridCells, "/path_planner/c_space", qos)
        self.robot_path = self.create_publisher(Path,'/path_planner/robot_path', qos)

        #self.create_service(GetPlan, 'path_planner/get_path', self.plan_path, qos)
        self.get_logger().info("path planner node initalized!")

        # numpy array of cell values
        self.map = np.zeros((0,0))

        # TODO: Setup TF Buffer and listener
        

    @staticmethod
    def obstacle_expansion(original_map:npt.NDArray[np.int32], padding:int):
        """
        Expands obstacles to be "padding" larger in all directions
        :param original_map     ndarray     Map of obstacles
        :param padding          int         Number of cells from obstacle to consider unsafe
        :return                 ndarray     Map of safe configuration space.
        """
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3,3))
        binary_map = (original_map == 100).astype(np.uint8)
        expanded_map = cv.dilate(binary_map, kernel, iterations=padding)

        return expanded_map
    
    @staticmethod
    def risk_map(expanded_map:npt.NDArray[np.int32]):

        pass

    def handle_map(self, map:OccupancyGrid):
        """
        Recieve raw map, convert to numpy array, expand obstacles.
        Save safe map and republish. 
        :param map      OccupancyGrid   The current map.
        """

        self.get_logger().info("map received")
        map_data = map.data
        self.mapinfo = map.info

        np_map = np.asarray(map_data)
        self.map = np.reshape(np_map, (self.mapinfo.height, self.mapinfo.width))

        turtlebot_radius = 0.105 # meters

        if turtlebot_radius < self.mapinfo.resolution:
            padding = 1
        else:
            padding = int(turtlebot_radius/self.mapinfo.resolution)
        self.get_logger().info(f"Iterations for padding: {padding}")

        expanded_map = self.obstacle_expansion(self.map, padding)

        expanded_map_uint8 = (expanded_map * 100).astype(np.int8)
        flattened_map = expanded_map_uint8.flatten()

        new_map = OccupancyGrid()
        new_map.info = map.info
        new_map.header.frame_id = 'map'
        new_map.data = flattened_map

        self.safe_map.publish(new_map)
        self.get_logger().info("published new driveable map on /safe/map")


    @staticmethod
    def grid_to_world(mapdata: MapMetaData, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [MapMetaData] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        resolution = mapdata.resolution # note: in meters

        realOriginX = mapdata.origin.position.x
        realOriginY = mapdata.origin.position.y

        realWorldX = p[0]*resolution + realOriginX + resolution/2
        realWorldY = p[1]*resolution + realOriginY + resolution/2

        pt = Point()
        pt.x = realWorldX
        pt.y = realWorldY
        pt.z =  0.0

        return pt

        
    @staticmethod
    def world_to_grid(mapdata: MapMetaData, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [MapMetaData] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        resolution = mapdata.resolution

        r_origin_x = mapdata.origin.position.x
        r_origin_y = mapdata.origin.position.y

        cell_x = int((wp.x - r_origin_x)/resolution)
        cell_y = int((wp.y - r_origin_y)/resolution)

        return (cell_x, cell_y)
    
    def clicked_point_callback(self, msg:Point):
        '''subscribe to rviz, tool, print'''


    def a_star(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Runs A* search from start coordinate to end coordinate using class member map.
        :param start    [[(int,int)]]     start coordinate in map    
        :param goal     [[(int,int)]]      goal coordinate in map    
        :return

        """
        # GROUP

        #use GraphNode and PriorityQueue
        pass


    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        return distance
        

    def build_path_message(self, path: list[tuple[int, int]]) -> Path:
        """
        Converts a list of cell coordinates to a Path Message
        :param path     [(int,int)]     The cell coordinates corresponding to the current map
        :return         Path            
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for grid_cell in path:
            pose_message = PoseStamped()
            pose_message.header.stamp = self.get_clock().now().to_msg()
            pose_message.header.frame_id = "map"
            pose_message.pose.position = PathPlanner.grid_to_world(self.mapinfo, grid_cell)

            path_msg.poses.append(pose_message)

        return path_msg


    def neighbors_of_4(self, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the safe 4-neighbors cells of (x,y) in the occupancy grid.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable in 4 cardinal directions.
        """
        x,y = p[0], p[1]
        height, width = self.map.shape
        locations = [(0,1),(0,-1),(1,0),(-1,0)]
        neighbors_4 = []

        for location in locations:
            new_location = (x + location[0], y + location[1])

            if new_location[0] < 0 or new_location[0] >= width or new_location[1] < 0 or new_location[1] >= height:
                continue

            if self.map[new_location[1], new_location[0]] != 100: 
                neighbors_4.append(new_location)

        return neighbors_4

        
    def neighbors_of_8(self, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the safe 8-neighbors cells of (x,y) in the occupancy grid.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable in all 8 directions.
        """

        x,y = p[0], p[1]
        height, width = self.map.shape
        n_of_4 = self.neighbors_of_4(p)
        more_locations = [(1,1),(-1,-1),(1,-1),(-1,1)]
        neighbors = []

        for location in more_locations:

            new_location = (x + location[0], y + location[1])

            if new_location[0] < 0 or new_location[0] >= width or new_location[1] < 0 or new_location[1] >= height:
                continue

            if self.map[new_location[1], new_location[0]] != 100: 
                neighbors.append(new_location)
            else: 
                continue
        
        return neighbors + n_of_4

    def get_edge_cost(self, p1: tuple[int, int],p2: tuple[int, int]):
        """
        Compute cost to traverse between 2 nodes.
        :param p1       [(int, int)]    The coordinate in the grid of start.
        :param p2       [(int, int)]    The coordinate in the grid of end.
        """

        # INDIVIDUAL
        pass

    def draw_visited(self,visited):
        """
        Draw set of visited node as GridCells message.
        """
        cells = GridCells()
        cells.cell_width = self.map.info.resolution
        cells.cell_height = self.map.info.resolution
        cells.header = self.map.header
        for index in visited:
            p = self.grid_to_world(self.map,index[:2])            
            cells.cells.append(p)
        self.visited_publisher.publish(cells)

    def build_path(self,final_node: GraphNode):
        """
        Given goal node from search tree, build path from start to goal to get there
        """

        current_node = final_node
        path = []
        while current_node is not None:
            path.append(current_node.pose)
            current_node= current_node.parent
        return path[::-1]

    def plan_path(self, request, response): 
        """
        Plans a path between the current pose and the goal message locations.
        Internally uses A* to plan the optimal path.
        :param request  nav_msgs.srv._get_plan.GetPlan_Request  Start and End Pose for plan
        :param response nav_msgs.srv._get_plan.GetPlan_Response 
        :return         nav_msgs.srv._get_plan.GetPlan_Response     
        """

        # TODO: Add error handling for if there is no map available
        
        # TODO: Calculate a path using A* 

        # TODO: Return your path 

        pass

    @staticmethod
    def optimize_path(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """   
        # EXTRA CREDIT
        pass


    def neighbors_with_orientation(self,p: tuple[int, int,int]) -> list[tuple[int, int, int]]:
        """
        Returns the safe neighbours for nonholonomic robot with pose x,y,theta.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int, int)]    The coordinate in the grid.
        :return        [[(int,int, int)]]   A list of walkable 8-neighbors.
        """
        # EXTRA CREDIT      
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()