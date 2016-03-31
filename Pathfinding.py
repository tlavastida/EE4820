import astar
import numpy
import time

class PathFinder():
    def __init__(self,map):
        self.map = map
        self.path = []

    #inputs - current pos, target pos
    #outputs - furthest position in a straight line
    def findPath(self, current, target):
        if not self.path or current not in self.path:
            self.path = astar.astar(self.map, current, target)
        
        #get only turning points of the path
        new_path = [current] #keep the first element since it will always be needed
        for nodeindex in range(1, len(self.path)-1):
            direction1 = (self.path[nodeindex][0] - self.path[nodeindex-1][0], self.path[nodeindex][1] - self.path[nodeindex-1][1])
            direction2 = (self.path[nodeindex+1][0] - self.path[nodeindex][0], self.path[nodeindex+1][1] - self.path[nodeindex][1])
            if direction1 != direction2:
                new_path.append(self.path[nodeindex])
        new_path.append(target) #self.path[-1]) #add in the last element
        self.path = new_path
        print(self.path)

def test():
    
    nmap = numpy.array([
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #fifthloc: (1,19)
            [1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,0, 0,0,1], 

            [1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,0, 0,0,1],
            [1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,0, 0,0,1], #sixthloc: (4,1), fourthloc: (4,22)
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],

            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],

            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],


            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #thirdloc: (13,2)
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],

            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #secondloc: (16,2)
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],

            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #firstloc: (19,22)
            [1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],

            [1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #start: (22, 2)
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1]])

    testing = PathFinder(nmap)

    find0 = testing.findPath((22,2), (19,22))
   
    #find1 = testing.findPath((19,22), (16,2))

    #find2 = testing.findPath((16,2), (13,2))

    find3 = testing.findPath((13,2), (4,22))

    #find4 = testing.findPath((4,22), (1,19))

    #find5 = testing.findPath((1,19), (4,1))

    #find6 = testing.findPath((4,1), (22,2))

start = time.clock()
test()
end = time.clock()
print(end-start)



