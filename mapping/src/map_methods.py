#!/usr/bin/env python3
import math
from typing import Tuple, List
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from rclpy.node import Node

class MapMethods(Node):
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
                index = MapMethods.grid_to_index(mapdata, coord)
                occVal = mapdata.data[index]
                freeThreshold = 0.196
                if (occVal < freeThreshold and occVal !=-1): 
                    return True
                else:
                    return False
            else:
                return False
            
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

            if MapMethods.is_cell_walkable(mapdata, (x+1, y)):
                neighborCells8.append((x+1, y))
            if MapMethods.is_cell_walkable(mapdata, (x-1, y)):
                neighborCells8.append((x-1, y))
            if MapMethods.is_cell_walkable(mapdata, (x, y+1)):
                neighborCells8.append((x, y+1))
            if MapMethods.is_cell_walkable(mapdata, (x, y-1)):
                neighborCells8.append((x, y-1))
            if MapMethods.is_cell_walkable(mapdata, (x+1, y+1)):
                neighborCells8.append((x+1, y+1))
            if MapMethods.is_cell_walkable(mapdata, (x+1, y-1)):
                neighborCells8.append((x+1, y-1))
            if MapMethods.is_cell_walkable(mapdata, (x-1, y+1)):
                neighborCells8.append((x-1, y+1))
            if MapMethods.is_cell_walkable(mapdata, (x-1, y-1)):
                neighborCells8.append((x-1, y-1))

            return neighborCells8
        
        @staticmethod
        def expanded_neighbors8(p: Tuple[int, int], c_space: int):
            x, y = p[0], p[1]
            radius = c_space + 4
            expanded_neighbors = []

            for dx, dy in [(1,1)]: # (-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),
                    dx, dy = dx * radius, dy * radius
                    new_point = (x+dx, y+dy)
                    expanded_neighbors.append(new_point)
            return expanded_neighbors
