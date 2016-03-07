import numpy
from heapq import *
import time

start = time.clock()
class FindPath:
    """description of class"""
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

    def heuristic(self, a, b):
        #return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 # Euclidean distance, squared
        return (abs(a[0] - b[0]) + abs(a[1] - b[1]))    # Manhattan distance  

    def astar(self, array, start, goal):

        neighbors = [(0,1),(0,-1),(1,0),(-1,0)] # ,(1,1),(1,-1),(-1,1),(-1,-1)]

        close_set = set()
        came_from = {}    
        gscore = {start:0}
        fscore = {start:self.heuristic(start, goal)}
        oheap = []

        heappush(oheap, (fscore[start], start))
    
        while oheap:

            current = heappop(oheap)[1]

            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.reverse() # start to goal, not goal to start
                # data.append(current)
                return data

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j            
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
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
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heappush(oheap, (fscore[neighbor], neighbor))
                
        return False
end = time.clock()
print(end-start)

"""
class CourseMap:
    #map of the course
    nmap = numpy.array([
        [0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,0,0],
        [1,1,1,1,1,1,1,0],
        [1,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,1],
        [0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,1],
        [0,0,0,0,0,0,0,0],
        [1,0,1,1,1,1,1,1],
        [0,0,0,0,0,0,0,0]])

    def FindPath():
        #takes AstarAlg and CourseMap to find the shortest path

    def CheckPosition(path):
        #if node not in the list of nodes for the path to goal, find path to goal from the current node
        for i in map:
            for x in path:
                if i == x:
                    #use AstarAlg
            return path


def Runcourse():
    
"""