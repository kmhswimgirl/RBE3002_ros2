#!/usr/bin/env python3

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
        self.goal_sent = False

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

    def extract_frontier_seeds(self, mapdata: OccupancyGrid) -> List[Tuple[int,int]]:
        ''' return all cells that border free and unknown space'''
        seeds = []
        w, h = mapdata.info.width, mapdata.info.height
        for y in range(h):
            for x in range(w):
                if not MapMethods.is_cell_walkable(mapdata, (x,y)): continue
                for dx, dy in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < w and 0 <= ny < h and mapdata.data[MapMethods.grid_to_index(mapdata,(nx,ny))] == -1:
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
            if num_gc >= self.frontier_threshold:
                new_frontiers.append(cluster)
        return new_frontiers
    
    def get_largest_frontier(self, frontiers:List[List[Tuple[int,int]]]):
        if not frontiers:
            return None
        sizes = [len(cluster) for cluster in frontiers]
        largest_i = sizes.index(max(sizes))
        largest_cluster = frontiers[largest_i]
        cx, cy = self.calc_centroid(largest_cluster)
        # return centroid in grid-cell coordinates
        self.loginfo(f'[DEBUG]: get_largest_frontier centroid = ({cx:.3f}, {cy:.3f})')
        return (cx, cy)
    
    def navigation_update(self):

        if self.mapdata is None:
            self.loginfo('No Map!')
            return

        # get cells that are next to freespace & unknown
        seeds = self.extract_frontier_seeds(self.mapdata)

        fc = GridCells(
            header= self.mapdata.header,
            cell_width= self.mapdata.info.resolution,
            cell_height= self.mapdata.info.resolution,
            cells=[MapMethods.grid_to_world(self.mapdata, s) for s in seeds]
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
            m.lifetime.sec = 8
            ma.markers.append(m)

        largest = self.get_largest_frontier(cleaned_clusters)

        if largest is None:
            self.loginfo('NAV GOAL: no frontiers found')
        else:
            gx, gy = largest
            # convert grid centroid -> world meters
            res = self.mapdata.info.resolution
            origin = self.mapdata.info.origin
            world_x = gx * res + origin.position.x
            world_y = gy * res + origin.position.y

            pose = self.convert_to_pose((1.0, 1.0))
            self.get_logger().info(f'NAV GOAL (world): ({world_x:.3f}, {world_y:.3f})')
            if not self.goal_sent:
                self.nav_goal.publish(pose)
                self.goal_sent = True
        
        self.frontiers_rviz.publish(fc)
        self.centroids.publish(ma)

    def convert_to_pose(self, input_coord: Tuple):
        pose_message = PoseStamped()
        pose_message.header.stamp = self.get_clock().now().to_msg()
        pose_message.header.frame_id = 'map'
        pose_message.pose.position.x = float(input_coord[0])
        pose_message.pose.position.y = float(input_coord[1])

        return pose_message

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