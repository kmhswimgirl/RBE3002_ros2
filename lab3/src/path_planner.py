#!/usr/bin/env python3

# RBE 3002: Lab 3 (Jazzy Jalsico version)

import math
import rclpy

from rclpy.node import Node
from nav_msgs.srv import GetPlan, GetMap, GetPlan_Response
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from typing import List, Tuple
from queue import PriorityQueue
from rclpy.qos import QoSProfile, DurabilityPolicy

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner') # initialize node
        self.loginfo = self.get_logger().info

        # QoS config for c-space ?
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # add visuals to rviz
        self.expanded_cells = self.create_publisher(GridCells, "/expanded_cells", qos) 
        self.frontier_cells = self.create_publisher(GridCells, "/frontier", qos)
        self.c_space = self.create_publisher(GridCells, "/path_planner/c_space", qos)
        self.robot_path = self.create_publisher(Path,'/robot_path', qos)

        # test subscriber for checking a* accuracy
        self.rviz_goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.rviz_2d_nav_goal, 10)

        # service call for generating path
        self.path_to_drive = self.create_service(GetPlan, '/path_planner/plan_path', self.plan_path)

        # c space test -- remove later
        mapdata = self.request_map()
        if mapdata is not None:
            self.calc_cspace(mapdata, 1)

    def rviz_2d_nav_goal(self, msg:PoseStamped):
        self.loginfo(f"Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        mapdata = self.request_map()
        if mapdata is None: # error handling for the map request failing
            self.loginfo("map not found")
            return
        
        c_space_info = self.calc_cspace(mapdata, 1)
        st = self.world_to_grid(mapdata, msg.pose.position.)
        go = self.world_to_grid(mapdata, msg.pose.position)
        pth = self.a_star(c_space_info, st, go)
        self.path_to_message(mapdata,pth)

        


    # --------------------- easy stuff, should not need any ros2 translating
    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: Tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        #variables
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

        # since ros2 is annoying, have to set each point individually 
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
            index = PathPlanner.grid_to_index(mapdata, coord)
            occVal = mapdata.data[index]
            freeThreshold = 0.196
            if (occVal < freeThreshold and occVal !=-1): 
                return True
            else:
                return False
        else:
            return False
    
    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        x, y = p[0], p[1]

        neighborCells4 = []

        if PathPlanner.is_cell_walkable(mapdata, (x+1, y)):
            neighborCells4.append((x+1, y))
        if PathPlanner.is_cell_walkable(mapdata, (x-1, y)):
            neighborCells4.append((x-1, y))
        if PathPlanner.is_cell_walkable(mapdata, (x, y+1)):
            neighborCells4.append((x, y+1))
        if PathPlanner.is_cell_walkable(mapdata, (x, y-1)):
            neighborCells4.append((x, y-1))

        return neighborCells4
    
    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        
        x,y = p[0], p[1]

        neighborCells8 = []

        if PathPlanner.is_cell_walkable(mapdata, (x+1, y)):
            neighborCells8.append((x+1, y))
        if PathPlanner.is_cell_walkable(mapdata, (x-1, y)):
            neighborCells8.append((x-1, y))
        if PathPlanner.is_cell_walkable(mapdata, (x, y+1)):
            neighborCells8.append((x, y+1))
        if PathPlanner.is_cell_walkable(mapdata, (x, y-1)):
            neighborCells8.append((x, y-1))
        if PathPlanner.is_cell_walkable(mapdata, (x+1, y+1)):
            neighborCells8.append((x+1, y+1))
        if PathPlanner.is_cell_walkable(mapdata, (x+1, y-1)):
            neighborCells8.append((x+1, y-1))
        if PathPlanner.is_cell_walkable(mapdata, (x-1, y+1)):
            neighborCells8.append((x-1, y+1))
        if PathPlanner.is_cell_walkable(mapdata, (x-1, y-1)):
            neighborCells8.append((x-1, y-1))

        return neighborCells8
    
    
    # ------------------- medium difficulty, might need a few tweaks

    def path_to_poses(self,mapdata: OccupancyGrid, path: List[Tuple[int, int]]) -> List[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        world_path = []

        for grid_cell in path:
            pose_message = PoseStamped()
            pose_message.header.stamp = self.get_clock().now().to_msg()
            pose_message.header.frame_id = "map"
            pose_message.pose.position = PathPlanner.grid_to_world(mapdata, grid_cell)

            world_path.append(pose_message)

        return world_path

    
    # ---------------------- potential full re-write due to ros2 syntax/logic

    def request_map(self) -> OccupancyGrid: # this was a bitch to figure out
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        self.loginfo("requesting map")

        map_client = self.create_client(GetMap, "/map_server/map") # ros2 service list to check for topic. totally got that wrong last time lol
        while not map_client.wait_for_service(1):
            self.loginfo("mapdata not available")
        
        map_request = GetMap.Request()
        map_async_call = map_client.call_async(map_request)
        rclpy.spin_until_future_complete(self, map_async_call) # wait for the map server service call to arrive

        mapdata = map_async_call.result()

        if mapdata is not None:
            self.loginfo("map retrieved")
            return mapdata.map
        else:
            return None
        
    # ------------------ C SPACE & A STAR

    def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        grid = mapdata.data
        addedCells = []
        paddedGrid = list(grid)
        arrayRows = mapdata.info.height
        arrayCols = mapdata.info.width

        self.loginfo("Calculating C-Space")  
        x, y = 0, 0

        for cell in range(len(paddedGrid) - 1):

            if not PathPlanner.is_cell_walkable(mapdata, (x,y)):

                for nextY in range(-padding, padding + 1): # Stop value isn't included in output so need to add one
                    for nextX in range(-padding, padding + 1):
                        if((x+nextX) in range(0, arrayCols) and (y+nextY) in range(0, arrayRows) and PathPlanner.is_cell_walkable(mapdata, (x+nextX, y+nextY))): # Ensure that neighboring cell is walkable, should also be fine to check if cell is within the image/grid
                            addedCells.append(PathPlanner.grid_to_world(mapdata, (x+nextX, y+nextY))) #Add the real world version of the cell to the list
                            paddedGrid[PathPlanner.grid_to_index(mapdata, (x+nextX, y+nextY))] = 100 # Make the cell unwalkable in the new grid
                
            x += 1
            if x % arrayCols == 0:
                y += 1
                x = 0

        paddedCells = GridCells()
        paddedCells.header = mapdata.header
        paddedCells.cell_width = mapdata.info.resolution 
        paddedCells.cell_height = mapdata.info.resolution
        paddedCells.cells = addedCells
        paddedCells.header.frame_id = "map"
        
        self.c_space.publish(paddedCells)

        self.loginfo(f"Publishing {len(addedCells)} C-space cells")

        # more ros2 type annoyances...
        c_map = OccupancyGrid()
        c_map.header = mapdata.header
        c_map.info = mapdata.info
        c_map.data = paddedGrid

        return c_map
    
    
    def a_star(self, mapdata: OccupancyGrid, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        ### REQUIRED CREDIT
        self.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        frontier = PriorityQueue()
        frontier.put((0, start)) # Start comes from function input
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        visited = []
        path = [] # Store the optimal path to get from start to goal

        visitedCells = GridCells()
        visitedCells.header = mapdata.header
        visitedCells.header.frame_id = "map"
        visitedCells.cell_width = mapdata.info.resolution
        visitedCells.cell_height = mapdata.info.resolution

        pathCells = GridCells()
        pathCells.header = mapdata.header
        pathCells.header.frame_id = "map"
        pathCells.cell_width = mapdata.info.resolution
        pathCells.cell_height = mapdata.info.resolution

        frontierCells = GridCells()
        frontierCells.header = mapdata.header
        frontierCells.header.frame_id = "map"
        frontierCells.cell_width = mapdata.info.resolution
        frontierCells.cell_height = mapdata.info.resolution

        # Run while there are still points in the frontier
        while not frontier.empty():
            _, current = frontier.get()

            if current == goal:
                break

            # Get all of the neighbors of 8 and check if they should be added
            for next in PathPlanner.neighbors_of_8(mapdata, current):
                new_cost = cost_so_far[current] + (abs(current[0] - next[0]) + abs(current[1] - next[1])) # Calculate the Manhattan distance to get from current point to next point and add it to cost
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(next, goal) # Use Euclidean distance function from the current point to the new point for the heuristic
                    frontier.put((priority, next)) # Put the next point in the frontier
                    came_from[next] = current
                    frontierCells.cells.append(PathPlanner.grid_to_world(mapdata, next))
            
            # Also store which cell each one came from
            visited.append(current) # Add cell to the visited list
            visitedCells.cells.append(PathPlanner.grid_to_world(mapdata, current)) # Add world coordinate version of cell to be shown in Rviz

        # If the goal was not actually reached, return empty path
        if goal not in came_from:
            return []

        path = []
        current = goal

        # Make the path based off the current point and where it came from (create list of points backwards)
        while current is not None:
            path.append(current)
            if current == start:
                break
            current = came_from.get(current)

        path.reverse() # Cells were put in backwards so reverse list


        # Publish the expanded cells to be visualized in Rviz
        self.expanded_cells.publish(visitedCells)

        return path # Return the robot's path, in grid coordinates


    # ---------- PATH STUFF

    def path_to_message(self, mapdata: OccupancyGrid, path: List[Tuple[int, int]]) -> Path:
            """
            Takes a path on the grid and returns a Path message.
            :param path [[(int,int)]] The path on the grid (a list of tuples)
            :return     [Path]        A Path message (the coordinates are expressed in the world)
            """
            ### REQUIRED CREDIT
            self.loginfo("Returning a Path message")

            # creating the world_path_message from the previous path 
            world_path_message = Path() # Create a message of type Path()    
            world_path_message.header.stamp = self.get_clock().now().to_msg()
            world_path_message.header.frame_id = "map"
            world_path_message.poses = self.path_to_poses(mapdata, path)
            self.robot_path.publish(world_path_message) #Publish the message to be shown in Rviz

            # service call response to go to the driver node
            path_response = GetPlan_Response()
            path_response.plan = world_path_message

            self.loginfo(f"Publishing path: {len(world_path_message.poses)} poses")
            return path_response
            

    def plan_path(self, request, response):
            """
            Plans a path between the start and goal locations in the requested.
            Internally uses A* to plan the optimal path.
            :param req 
            """
            
            mapdata = self.request_map()
            if mapdata is None:
                return Path()
            
            cspacedata = self.calc_cspace(mapdata, 1)

            start = PathPlanner.world_to_grid(mapdata, request.start.pose.position)
            goal  = PathPlanner.world_to_grid(mapdata, request.goal.pose.position)
            path  = self.a_star(cspacedata, start, goal)

            return self.path_to_message(mapdata, path)
            
            
    # --------------- EX. CREDIT ----------------------
    @staticmethod
    def optimize_path(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        #self.loginfo("Optimizing path")
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