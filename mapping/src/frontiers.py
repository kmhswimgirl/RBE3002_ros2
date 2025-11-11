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
from rclpy.executors import MultiThreadedExecutor

class Frontiers(Node):
    def __init__(self):
        super().__init__('frontiers')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('grid_square_threshold', 15),
                ('c_space', 2),
                ('c_space_buffer', 0),
                ('replan_rate', 10.0)
            ])

        self.loginfo = self.get_logger().info

        # frontier / centroid tuning
        self.frontier_threshold = self.get_parameter('grid_square_threshold').value
        self.c_space = self.get_parameter('c_space').value
        self.replan_rate = self.get_parameter('replan_rate').value
        self.buffer = self.get_parameter('c_space_buffer').value

        # quality of service stuff
        qos = QoSProfile(depth=100) # practically queue size lol
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.mapdata = None
        self.goal_sent = False
        self.phase_2 = False
        self.current_goal = None
        
        # need map and nav update
        # --> CB group, same as L2
        self.map_timer = ReentrantCallbackGroup()

        # map handling
        self.map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos, callback_group=self.map_timer)

        self.send_goal = self.create_timer(self.replan_rate, self.toggle_goal_send, callback_group=self.map_timer)

        # timer calls update nav points, which recalculates the centroids + frontiers
        self.update_nav_points = self.create_timer(1.0, self.navigation_update, callback_group=self.map_timer)

        # publish the calculated nav_goal
        self.nav_goal = self.create_publisher(PoseStamped, '/nav_goal', qos) # this becomes the goal in control

        # rviz debug
        self.frontiers_rviz = self.create_publisher(GridCells, '/frontier_cells', qos)
        self.centroids = self.create_publisher(MarkerArray, '/centroids', qos)
        self.drive_points = self.create_publisher(MarkerArray, 'frontiers/drive_points', qos)

    def map_callback(self, msg: OccupancyGrid): # maybe working function to use whenever the map gets changes (or when you re call the plan_path service)
        '''Update the whenever /map recieves a new message'''
        self.mapdata = msg

    def toggle_goal_send(self):
        '''toggle boolean when ready to send a new nav goal'''
        if self.phase_2 == False:
            self.loginfo("goal being published")
            self.nav_goal.publish(self.current_goal)

    ## ------------- Frontier Finding Functions ------------- ##

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
    
    ## --------------- Frontier Scoring Functions --------------- ##
    def filter_frontiers(self, frontiers: List[List[Tuple[int,int]]]):
        '''remove clusters that are either SLAM mistakes or are too small to be considered a viable nav goal'''
        new_frontiers = []
        for cluster in frontiers:
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
        cx, cy = int(cx), int(cy)

        # return centroid in grid coordinates
        self.loginfo(f'[DEBUG]: get_largest_frontier | centroid = ({cx}, {cy})')

        exp_cells = self.calc_drive_goals(self.mapdata, largest_cluster, self.buffer)

        self.loginfo(f'drivable coordinates??? {exp_cells}')

        if exp_cells is None:
            self.loginfo('[DEBUG]: get_largest_frontier | no cells...')
            return
        return exp_cells

    ## ------------------- 
    def navigation_update(self):
        '''update the map with frontiers, centroids, & drive goals'''

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

        ma = MarkerArray() # visualizing in Rviz
        for i, clusters in enumerate(cleaned_clusters):
            score = self.score_wh_ratio(clusters)
            cx, cy = self.calc_centroid(clusters)
            cl_length = self.frontier_length(clusters)

            i_cx, i_cy = int(cx), int(cy)
  
            self.loginfo(f'Cluster #{i}| ({i_cx},{i_cy}) | # of gc:{len(clusters)} | length:{cl_length} | score: {score}') # debug

            m = Marker(
                header=self.mapdata.header,
                ns='frontier_centroids',
                id=i,
                type=Marker.SPHERE,
                action=Marker.ADD
            )

            m.pose.position.x = cx*self.mapdata.info.resolution + self.mapdata.info.origin.position.x
            m.pose.position.y = cy*self.mapdata.info.resolution + self.mapdata.info.origin.position.y
            m.pose.position.z = 0.01
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r = 1.0; m.color.a = 0.8
            m.lifetime.sec = 8
            ma.markers.append(m)

        largest = self.get_largest_frontier(cleaned_clusters)

        if largest is None or len(largest) == 0: # catch if there are no frontiers left
            self.loginfo('NAV GOAL: no frontiers found')
        else:
            gx, gy = largest[0]

            # convert grid centroid -> world meters
            res = self.mapdata.info.resolution
            origin = self.mapdata.info.origin
            world_x = gx * res + origin.position.x
            world_y = gy * res + origin.position.y

            self.current_goal = self.convert_to_pose((world_x, world_y))
            self.get_logger().info(f'NAV GOAL (world): ({world_x:.3f}, {world_y:.3f})')
        
        self.frontiers_rviz.publish(fc)
        self.centroids.publish(ma)

    def convert_to_pose(self, input_coord: Tuple):
        pose_message = PoseStamped()
        pose_message.header.stamp = self.get_clock().now().to_msg()
        pose_message.header.frame_id = 'map'
        pose_message.pose.position.x = float(input_coord[0])
        pose_message.pose.position.y = float(input_coord[1])

        return pose_message
        
    def calc_drive_goals(self, map:OccupancyGrid, cluster:List[Tuple[int,int]], buffer:float):
        drive_goals = []
        mk = MarkerArray()
        x2, y2 = cluster[-1]
        x1, y1 = Frontiers.calc_centroid(cluster)

        dx21 = x1 - x2
        dy21 = y1 - y2

        d12 = math.sqrt( dx21**2 + dy21**2 )
        d13 = self.c_space + buffer

        psi = d13/d12

        dy13 = int(-psi * dx21)
        dx13 = int(psi * dy21)

        x3 = int(x1 + dx13)
        y3 = int(y1 + dy13)

        x4 = int(x1 - dx13)
        y4 = int(y1 - dy13)

        coords = [(x3,y3), (x4,y4)]

        for coord in coords:
            if MapMethods.is_cell_walkable(map, coord):
                m = Marker(
                    header=self.mapdata.header,
                    ns='drive_points',
                    id=coords.index(coord),
                    type=Marker.SPHERE,
                    action=Marker.ADD
                )

                px, py = coord[0], coord[1]
                m.pose.position.x = px*self.mapdata.info.resolution + self.mapdata.info.origin.position.x
                m.pose.position.y = py*self.mapdata.info.resolution + self.mapdata.info.origin.position.y
                m.pose.position.z = 0.01
                m.scale.x = m.scale.y = m.scale.z = 0.08
                m.color.g = 1.0; m.color.a = 0.8
                m.lifetime.sec = 8

                mk.markers.append(m)
                drive_goals.append(coord)

        self.drive_points.publish(mk)     
        return drive_goals
    
    def score_wh_ratio(self, cluster:List[Tuple[int, int]]):
    
        max_x = max(cluster, key=lambda x: x[0])[0]
        min_x = min(cluster, key=lambda x: x[0])[0]

        max_y = max(cluster, key=lambda x: x[1])[1]
        min_y = min(cluster, key=lambda x: x[1])[1]

        dx = abs(max_x - min_x)
        dy = abs(max_y - min_y)

        if dy != 0:
            ratio = dx/dy
        else:
            ratio = dy/dx
        
        return ratio

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
    
    # def is_in_c_space(self, cspace:GridCells, drive_points:List[Tuple[int, int]]):


def main(args=None): # i think i got this right? idk
    rclpy.init(args=args)
    node = Frontiers()
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