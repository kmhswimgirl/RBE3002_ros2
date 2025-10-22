#!/usr/bin/env python3

import rclpy
import math
from typing import List, Tuple
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class Frontiers(Node):
    def __init__(self):
        super().__init__('frontiers')
        self.loginfo = self.get_logger().info

        # frontier / centroid tuning
        self.frontier_threshold = 12

        # quality of service stuff
        qos = QoSProfile(depth=100) # practically queue size lol
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.mapdata = None

        # need map and nav update
        # --> CB group, same as L2
        self.map_timer = ReentrantCallbackGroup()

        # map handling
        self.map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos, callback_group=self.map_timer)

        # timer calls update nav points, which recalculates the centroids + frontiers
        self.update_nav_points = self.create_timer(1.0, self.navigation_update, callback_group=self.map_timer)

        # publish the calculated nav_goal
        self.nav_goal = self.create_publisher(PoseStamped, '/nav_goal', qos) # this becomes the goal in control

        # rviz debug
        self.frontiers_rviz = self.create_publisher(GridCells, '/frontier_cells', qos)
        self.centroids = self.create_publisher(MarkerArray, '/centroids', qos)

    def map_callback(self, msg: OccupancyGrid): # maybe working function to use whenever the map gets changes (or when you re call the plan_path service)
        """
        Update the whenever /map recieves a new message
        """
        self.mapdata = msg
        self.loginfo('map updated!')

    def extract_frontier_seeds(self, mapdata: OccupancyGrid) -> List[Tuple[int,int]]:
        ''' return all cells that border free and unknown space'''
        seeds = []
        w, h = mapdata.info.width, mapdata.info.height
        for y in range(h):
            for x in range(w):
                if not Frontiers.is_cell_walkable(mapdata, (x,y)): continue
                for dx, dy in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < w and 0 <= ny < h and mapdata.data[Frontiers.grid_to_index(mapdata,(nx,ny))] == -1:
                        seeds.append((x,y))
                        break
        return seeds
    
    def cluster_frontiers(self, seeds: List[Tuple[int,int]]) -> List[List[Tuple[int,int]]]:
        '''figure out which seeds/ frontier cells are next to each other'''
        visited, clusters = set(), []
        seed_set = set(seeds)
        for s in seeds:
            if s in visited: continue
            stack, comp = [s], []
            while stack:
                p = stack.pop()
                if p in visited: continue
                visited.add(p)
                comp.append(p)
                x, y = p
                for dx, dy in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
                    q = (x+dx, y+dy)
                    if q in seed_set and q not in visited:
                        stack.append(q)
            clusters.append(comp)
        return clusters
    
    def filter_frontiers(self, frontiers: List[List[Tuple[int,int]]]):
        '''remove clusters that are either SLAM mistakes or are too small to be considered a viable nav goal'''
        new_frontiers = []
        for cluster in frontiers:
             # frontier_length = Frontiers.frontier_length(cluster) something off about the math...
            num_gc = len(cluster)
            if num_gc >= self.frontier_threshold: # if the number of
                new_frontiers.append(cluster)
        return new_frontiers
                
    def navigation_update(self):
        '''
        called on a timer. not sure what update rate would be good...
        '''
        if self.mapdata is None:
            self.loginfo('No Map!')
            return
        
        # get cells that are next to freespace & unknown
        seeds = self.extract_frontier_seeds(self.mapdata)

        fc = GridCells(
            header= self.mapdata.header,
            cell_width= self.mapdata.info.resolution,
            cell_height= self.mapdata.info.resolution,
            cells=[Frontiers.grid_to_world(self.mapdata, s) for s in seeds]
        )
        fc.header.frame_id = 'map'

        # confirming the number of clusters pre and post filtering 
        clusters = self.cluster_frontiers(seeds)
        num_of_clusters = len(clusters)
        self.loginfo(f'not filtered: {num_of_clusters}')

        cleaned_clusters = self.filter_frontiers(clusters)
        num_clusters_post = len(cleaned_clusters)
        self.loginfo(f'filtered: {num_clusters_post}')

        ma = MarkerArray()
        for i, clusters in enumerate(cleaned_clusters):
            cx, cy = self.calc_centroid(clusters)
            cl_length = self.frontier_length(clusters)
            self.loginfo(f'Cluster #{i}| ({cx},{cy}) | # of gc:{len(clusters)} | length:{cl_length}') # debug
            m = Marker(
                header=self.mapdata.header,
                ns='frontier_centroids',
                id=i,
                type=Marker.SPHERE,
                action=Marker.ADD
            )

            m.pose.position.x = cx*self.mapdata.info.resolution + self.mapdata.info.origin.position.x
            m.pose.position.y = cy*self.mapdata.info.resolution + self.mapdata.info.origin.position.y
            m.pose.position.z = 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.08
            m.color.r = 1.0; m.color.a = 0.8
            ma.markers.append(m)

        self.frontiers_rviz.publish(fc)
        self.centroids.publish(ma)
        
    @staticmethod
    def calc_centroid (cluster: List[Tuple[int,int]]) -> Tuple[float,float]:
        cx = sum(x for x,_ in cluster)/len(cluster)
        cy = sum(y for _,y in cluster)/len(cluster)
        centroid_coords = (cx,cy)
        return centroid_coords
    
    @staticmethod
    def frontier_length(cluster: List[Tuple[int,int]]) -> float:
        a, b = cluster[0], cluster[-1]
        v1 = abs(b[0]-a[0])
        v2 = abs(b[1]-a[1])
        length = math.hypot(v1, v2)
        return length

## ---------- Remove later when i figure out how to import classes lol---------- ##
    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: Tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        # variables
        arrayCols = mapdata.info.width
        x, y = p[0], p[1]

        # logic
        if y == 0:
            index = x
        else: 
            index = arrayCols*y + x

        # output
        return index
    
    @staticmethod
    def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        return distance
    
    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: Tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        resolution = mapdata.info.resolution # note: in meters

        realOriginX = mapdata.info.origin.position.x
        realOriginY = mapdata.info.origin.position.y

        realWorldX = p[0]*resolution + realOriginX + resolution/2
        realWorldY = p[1]*resolution + realOriginY + resolution/2

        # since ros2 is annoying, have to set each parameter individually 
        pt = Point()
        pt.x = realWorldX
        pt.y = realWorldY
        pt.z =  0.0

        return pt
    
    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> Tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        resolution = mapdata.info.resolution

        realOriginX = mapdata.info.origin.position.x
        realOriginY = mapdata.info.origin.position.y

        cellX = int((wp.x - realOriginX)/resolution)
        cellY = int((wp.y - realOriginY)/resolution)

        return (cellX, cellY)
    
    @staticmethod
    def is_cell_walkable(mapdata:OccupancyGrid, p: Tuple[int, int]) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        arrayRows = mapdata.info.height
        arrayCols = mapdata.info.width
        coord = (p[0],p[1])

        # FIXED BOUNDS CHECK
        if ((p[0] >= 0 and p[0] < arrayCols) and (p[1] >= 0 and p[1] < arrayRows)):
            index = Frontiers.grid_to_index(mapdata, coord)
            occVal = mapdata.data[index]
            freeThreshold = 0.196
            if (occVal < freeThreshold and occVal !=-1): 
                return True
            else:
                return False
        else:
            return False

def main(args=None):
    rclpy.init(args=args)
    node = Frontiers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()