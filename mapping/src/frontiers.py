import rclpy
import math
from typing import List, Tuple
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import PoseStamped
from pathing.src.path_planner import PathPlanner # import specifically for A star functionality
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

class Frontiers:
    def __init__(self):
        super().__init__('frontiers')

        # QoS config for c-space?
        qos = QoSProfile(depth=100)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # map handling
        self.map = self.create_subscription(OccupancyGrid, '/map', self.map_callback)
        self.mapdata: OccupancyGrid = None

        # publish the calculated nav_goal
        self.nav_goal = self.create_publisher(PoseStamped, '/nav_goal', qos) # this becomes the goal in control

        # rviz debug
        self.frontiers_rviz = self.create_publisher(GridCells, '/frontier_cells', qos)

    def map_callback(self, msg: OccupancyGrid): # maybe working function to use whenever the map gets changes (or when you re call the plan_path service)
        """
        Update the whenever it changes
        """
        self.mapdata = msg
        self.publish_to_rviz()
        self.loginfo("Map updated")

    def extract_frontier_seeds(self, mapdata: OccupancyGrid) -> List[Tuple[int,int]]:
        seeds = []
        w, h = mapdata.info.width, mapdata.info.height
        for y in range(h):
            for x in range(w):
                if not PathPlanner.is_cell_walkable(mapdata, (x,y)): continue
                for dx, dy in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < w and 0 <= ny < h and mapdata.data[PathPlanner.grid_to_index(mapdata,(nx,ny))] == -1:
                        seeds.append((x,y))
                        break
        return seeds
    
    def cluster_frontiers(self, seeds: List[Tuple[int,int]]) -> List[List[Tuple[int,int]]]:
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
    
    def publish_to_rviz(self):
        seeds = self.extract_frontier_seeds(self.mapdata)

        fc = GridCells(
            header= self.mapdata.header,
            cell_width= self.mapdata.info.resolution,
            cell_height= self.mapdata.info.resolution,
            cells=[PathPlanner.grid_to_world(self.mapdata, s) for s in seeds]
        )
        fc.header.frame_id = 'map'

        self.frontier_cells_pub.publish(fc)

        clusters = self.cluster_frontiers(seeds)
        ma = MarkerArray()
        for i, cluster in enumerate(clusters):
            cx, cy = self._calc_centroid(cluster)
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
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r = 1.0; m.color.a = 0.8
            ma.markers.append(m)
        self.centroid_pub.publish(ma)

        
    @staticmethod
    def calc_centroid (cluster: List[Tuple[int,int]]) -> Tuple[float,float]:
        cx = sum(x for x,_ in cluster)/len(cluster)
        cy = sum(y for _,y in cluster)/len(cluster)
        centroid_coords = (cx,cy)
        return centroid_coords
    
    @staticmethod
    def frontier_length(cluster: List[Tuple[int,int]]) -> float:
        a, b = cluster[0], cluster[-1]
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        return length