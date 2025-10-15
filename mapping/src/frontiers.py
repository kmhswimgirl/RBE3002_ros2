import rclpy
import math
from pathing.src.path_planner import PathPlanner # import specifically for A star functionality

class Frontiers:
    def __init__(self):
        super().__init__('frontiers')

        # subscribers
            # map
        # publishers
            # all possible frontiers
                # - or - 
            # best frontier based on A* or euclidean distance

    # map callback (steal from pathing)

    # all frontier grid squares
        # find the squares that are marked as free 
        # that boarder unknown space
        # ---------------------------
        # for cell in map
            # if data in neighbor of 8 cell is "unknown"
            # append to list of frontier cells
    
    # condense frontiers / create the actual lines
        # take all frontier cells and combine them into groups
        # ----------------------------
        # for cell in frontier_cells
            # apparently this is just a dfs?
        