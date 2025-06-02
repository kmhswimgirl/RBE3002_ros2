#!/usr/bin/env python3

# RBE 3002: Lab 3 (Jazzy Jalsico version)

import math
import rclpy
from nav_msgs.srv import GetPlan, GetMap, GetPlanResponse
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from typing import List, Tuple
from priority_queue import PriorityQueue

class PathPlanner(Node):

    super().__init__('path_planner') # intialize node



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

        return Point(realWorldX, realWorldY, 0)
    
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

        index = PathPlanner.grid_to_index(mapdata, coord)
        occVal = mapdata.data[index]

        freeThreshold = 0.196
        
        if ((p[0] >= 0 and p[0] <= arrayCols) and (p[1] >= 0 and p[1] <= arrayRows)):
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
    @staticmethod
    def path_to_poses(mapdata: OccupancyGrid, path: List[Tuple[int, int]]) -> List[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        world_path = []

        for grid_cell in path:
            pose_message = PoseStamped()
            pose_message.header.stamp = rospy.Time.now() # need to replace with rclpy equvalent
            pose_message.header.frame_id = "map"
            pose_message.pose.position = PathPlanner.grid_to_world(mapdata, grid_cell)

            world_path.append(pose_message)

        return world_path
    
    # ---------------------- potential full re-write due to ros2 syntax/logic

    @staticmethod
    def request_map() -> OccupancyGrid:
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map") 

        # Try to connect to service
        rospy.wait_for_service('/static_map')

        # Try to get map from service and return error if the map could not be received
        try:
            getMapService = rospy.ServiceProxy('/static_map', GetMap)
            getMap = getMapService()
            rospy.loginfo("Got map")
            return getMap.map
        except:
            rospy.loginfo("Could not get map")
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

        rospy.loginfo("Calculating C-Space") # redo log statement 
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
        
        self.cspace.publish(paddedCells)

        return OccupancyGrid(mapdata.header, mapdata.info, paddedGrid)

    def a_star(self, mapdata: OccupancyGrid, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
            ### REQUIRED CREDIT
            rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

            frontier = PriorityQueue()
            frontier.put(start, 0) # Start comes from function input
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
            pathCells.header = mapdata.header
            pathCells.header.frame_id = "map"
            pathCells.cell_width = mapdata.info.resolution
            pathCells.cell_height = mapdata.info.resolution

            # Run while there are still points in the frontier
            while not frontier.empty():
                current = frontier.get()

                if current == goal:
                    break

                # Get all of the neighbors of 8 and check if they should be added
                for next in PathPlanner.neighbors_of_8(mapdata, current):
                    new_cost = cost_so_far[current] + (abs(current[0] - next[0]) + abs(current[1] - next[1])) # Calculate the Manhattan distance to get from current point to next point and add it to cost
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        priority = new_cost + PathPlanner.euclidean_distance(next, goal) # Use Euclidean distance function from the current point to the new point for the heuristic
                        frontier.put(next, priority) # Put the next point in the frontier
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
                current = came_from.get(current)
                if current == start:
                    path.append(start)
                    break 

            path.reverse() # Cells were put in backwards so reverse list

            # Publish the expanded cells to be visualized in Rviz
            self.expandedCells.publish(visitedCells)

            return path 

    # ---------- PATH STUFF

    def path_to_message(self, mapdata: OccupancyGrid, path: List[Tuple[int, int]]) -> Path:
            """
            Takes a path on the grid and returns a Path message.
            :param path [[(int,int)]] The path on the grid (a list of tuples)
            :return     [Path]        A Path message (the coordinates are expressed in the world)
            """
            ### REQUIRED CREDIT
            rospy.loginfo("Returning a Path message")
            world_path_message = Path() # Create a message of type Path()

            # Set fields of path message and use the path_to_poses function to get the poses the robot needs to be in to follow the path
            world_path_message.header.stamp = rospy.Time.now()
            world_path_message.header.frame_id = "map"
            world_path_message.poses = self.path_to_poses(mapdata, path)

            # Create a plan response that will get sent back to the client
            path_response = GetPlanResponse()
            path_response.plan = world_path_message

            self.robotPath.publish(world_path_message) #Publish the message to be shown in Rviz
            return path_response

    def plan_path(self, msg):
            """
            Plans a path between the start and goal locations in the requested.
            Internally uses A* to plan the optimal path.
            :param req 
            """
            mapdata = PathPlanner.request_map()
            if mapdata is None:
                return Path()
            
            cspacedata = self.calc_cspace(mapdata, 1)

            start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
            goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
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
        rospy.loginfo("Optimizing path")
        pass


