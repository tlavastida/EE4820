#!/usr/bin/env python3

# Author: Christian Careaga (christian.careaga7@gmail.com)
# A* Pathfinding in Python (2.7)
# Please give credit if used
# from http://code.activestate.com/recipes/578919-python-a-pathfinding-with-binary-heap/

import numpy
from heapq import *

def heuristic(a, b):
    #return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 #Euclidean distance, squared
    return (abs(a[0] - b[0]) + abs(a[1] - b[1]))    #Manhattan distance  

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0)] #,(1,1),(1,-1),(-1,1),(-1,-1)] #commented out diagonal adjacency nodes

    close_set = set() # closed_set just keeps track of nodes already observed; no need for list to sort
    came_from = {}    
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []        # oheap (open heap) is the list with the nodes traversed

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.reverse() # start to goal, not goal to start
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False

'''Here is an example of using my algo with a numpy array,
   astar(array, start, destination)
   astar function returns a list of points (shortest path)'''

# edited for our course (may need to change)
nmap = numpy.array([
    [0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,0,0],
    [1,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0]])
    
print (astar(nmap, (7,0), (1,0)))
