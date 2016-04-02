#pathfinding.py
#class for maintaining pathfinding data

import astar

class PathFinder:
    def __init__(self,grid):
        self.grid  = grid
        self.path = []


    def compute_path(start,goal):
        self.path = astar.astar(self.grid,start,goal)

    def
