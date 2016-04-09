import astar
import numpy
import time

class PathFinder:
    def __init__(self):
        self.start = None
        self.target = None
        self.map = numpy.array([
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
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #thirdloc: (13,1)
            [1,1,1, 1,1,1, 1,1,1, 1,0,0, 0,0,0, 0,0,0, 0,0,1, 1,1,1],

            [1,1,1, 1,1,1, 1,1,1, 1,0,0, 0,0,0, 0,0,0, 0,0,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #secondloc: (16,1)
            [1,1,1, 1,1,1, 1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,0,1, 1,1,1],

            [1,1,1, 1,1,1, 1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,0,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,1, 1,1,1, 1,1,1, 1,0,0, 0,0,1], #firstloc: (19,22), yellow: (19,1)
            [1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],

            [1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #start: (22, 1), red: (22,22)
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1]])
    
    #inputs - start pos, target pos
    #outputs - furthest position in a straight line
    def findCorners(self, start, target):

        self.path = []
        self.corners = []
        if self.map[start] == 1:
            print("INVALID START INPUT")
            #############################
            #first find nodes that are 0#
            #############################
            repath = [start]
            xrange = 23 - start[0]
            yrange = 23 - start[1]
            for xindex in range(xrange):
                for yindex in range(yrange):
                    if self.map[(start[0]+xindex, start[1]+yindex)] == 0:
                        repath.append((start[0]+xindex, start[1]+yindex))
                        #break
            for nxindex in range(start[0]):
                for nyindex in range(start[1]):
                    if self.map[(start[0]-xindex, start[1]-yindex)] == 0:
                        repath.append((start[0]-xindex, start[1]-yindex))
                        #break
            repath.append(target)

            #####################################################################
            #find the one with shortest distance from strayed position to itself#
            #####################################################################
            repath2 = [start]
            for points in range(1,len(repath)-1):
                #if 0 tuple is north or south from start
                if (repath[points][1] - repath[0][1]) == 0:
                    dist1a = abs(repath[points][0] - repath[0][0])
                    dist1b = abs(repath[points+1][0] - repath[0][0])
                    if dist1a < dist1b:
                        repath2.append(repath[points])
                        break
                    elif dist1a > dist1b:
                        repath2.append(repath[points+1])
                        break
                    else:
                        repath2.append(repath[points])
                        break
                #if 0 tuple is east or west from start
                elif (repath[points][0] - repath[0][0]) == 0:
                    dist2a = abs(repath[points][1] - repath[0][1])
                    dist2b = abs(repath[points+1][0] - repath[0][0])
                    if dist2a < dist2b:
                        repath2.append(repath[points])
                        break
                    elif dist2a > dist2b:
                        repath2.append(repath[points+1])
                        break
                    else:
                        repath2.append(repath[points])
                        break
                #diagonal
                else:
                    dist3a = abs(repath[points][0] - repath[0][0]) + abs(repath[points][1] - repath[0][1])
                    dist3b = abs(repath[points+1][0] - repath[0][0]) + abs(repath[points+1][0] - repath[0][0])
                    if dist3a < dist3b:
                        repath2.append((repath[points][0], start[1]))
                        repath2.append(repath[points])
                        break
                    elif dist3a > dist3b:
                        repath2.append((repath[points+1][0], start[1]))
                        repath2.append(repath[points+1])
                        break
                    else:
                        repath2.append((repath[points][0], start[1]))
                        repath2.append(repath[points])
                        break

                #if self.map[repath[points]] == 0:
                #    repath2.append(repath[points])
                #    break
            repath2.append(target) #self.path[-1]) #add in the last element
            
            #####################################################################################
            #get the final array from strayed point to valid point then from valid point to goal#
            #####################################################################################
            repath3 =repath2[:-1]
            self.path = astar.astar(self.map, repath2[1], repath2[-1])
            #####################################
            #get only turning points of the path#
            #####################################
            for nodeindex in range(1, len(self.path)-1):
                direction1 = (self.path[nodeindex][0] - self.path[nodeindex-1][0], self.path[nodeindex][1] - self.path[nodeindex-1][1])
                direction2 = (self.path[nodeindex+1][0] - self.path[nodeindex][0], self.path[nodeindex+1][1] - self.path[nodeindex][1])
                if direction1 != direction2:
                    repath3.append(self.path[nodeindex])
            repath3.append(repath2[-1])
            self.corners = repath3
            print(self.corners)
        elif not self.path or start == self.path[0]:
            print("VALID INPUTS")
            self.path = astar.astar(self.map, start, target)

            #####################################
            #get only turning points of the path#
            #####################################
            new_path = [start] #keep the first element since it will always be needed
            for nodeindex in range(1, len(self.path)-1):
                direction1 = (self.path[nodeindex][0] - self.path[nodeindex-1][0], self.path[nodeindex][1] - self.path[nodeindex-1][1])
                direction2 = (self.path[nodeindex+1][0] - self.path[nodeindex][0], self.path[nodeindex+1][1] - self.path[nodeindex][1])
                if direction1 != direction2:
                    new_path.append(self.path[nodeindex])
            new_path.append(target) #self.path[-1]) #add in the last element
            self.corners = new_path
            print(self.corners)

    ####################################################
    #returns the next action to do                     #
    # 'T:L' - turn left                                #
    # 'T:R' - turn right                               #
    # 'F:<distance>' - go forward <distance> grid units#
    ####################################################
    # def nextAction(self, start, target):

    def actionList(self, initfacing):
        if not self.corners:
            raise ValueError('Need to compute a path first!')
        
        move_list = []
        facing = ""
        direction = ""

        facing = initfacing

        for index in range(len(self.corners)-1):
            distance1 = abs(self.corners[index][1] - self.corners[index+1][1]) #horizontal distance
            distance2 = abs(self.corners[index][0] - self.corners[index+1][0]) #vertical distance

            #######################
            #which direction to go#
            #######################
            if self.corners[index][1] > self.corners[index+1][1]:
                direction = "West"
            elif self.corners[index][1] < self.corners[index+1][1]:
                direction = "East"
            elif self.corners[index][0] > self.corners[index+1][0]:
                direction = "North"
            else: # self.corners[index][0] > self.corners[index+1][0]:
                direction = "South"
            #print("direction: ", direction)

            ############################################################################################
            #determine robot action (turn or forward) based on orientation and direction it needs to go#
            ############################################################################################
            #go North
            if direction == "North":
                if facing == "North":
                    pass
                    #move_list.append('F:' + str(distance2))
                    #print("previously facing: ", facing)
                elif facing == "South": #180
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:ROTATE')
                    else:
                        move_list.append('T:CITY:ROTATE')
                    #move_list.append('F:' + str(distance2))
                    #print("previously facing: ", facing)
                elif facing == "East":
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:L')
                    else:
                        move_list.append('T:CITY:L')
                    #move_list.append('F:' + str(distance2))
                    #print("previously facing: ", facing) 
                elif facing == "West":
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:R')
                    else:
                        move_list.append('T:CITY:R')
                    #move_list.append('F:' + str(distance2))
                    #print("previously facing: ", facing)

                #if (self.corners[index] == (22,4) and self.corners[index+1] == (19,4)):
                #    move_list.append('F:C')
                #else:
                move_list.append('F:' + str(distance2))
                facing = direction
                #print("facing: ", facing)

            #go South
            elif direction == "South":
                if facing == "North": #180
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:ROTATE')
                    else:
                        move_list.append('T:CITY:ROTATE')
                    #move_list.append('F:' + str(distance2))
                    #print("previously facing: ", facing)
                elif facing == "South":
                    pass
                    #move_list.append('F:' + str(distance2))
                    #print("previously facing: ", facing)
                elif facing == "East":
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:R')
                    else:
                        move_list.append('T:CITY:R')
                    #move_list.append('F:' + str(distance2))
                    #print("previously facing: ", facing)
                elif facing == "West":
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:L')
                    else:
                        move_list.append('T:CITY:L')
                    #move_list.append('F:' + str(distance2))
                    #print("previously facing: ", facing)

                #if (self.corners[index] == (19,4) and self.corners[index+1] == (22,4)):
                #    move_list.append('F:C')
                #else:
                move_list.append('F:' + str(distance2))
                facing = direction
                #print("facing: ", facing)

            #go West
            elif direction == "West":
                if facing == "North":
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:L')
                    else:
                        move_list.append('T:CITY:L')
                    #move_list.append('F:' + str(distance1))
                    #print("previously facing: ", facing)
                elif facing == "South":
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:R')
                    else:
                        move_list.append('T:CITY:R')
                    #move_list.append('F:' + str(distance1))
                    #print("previously facing: ", facing)
                elif facing == "East": #180
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:ROTATE')
                    else:
                        move_list.append('T:CITY:ROTATE')
                    #move_list.append('F:' + str(distance1))
                    #print("previouslyfacing: ", facing)
                elif facing == "West":
                    pass
                    #move_list.append('F:' + str(distance1))
                    #print("previously facing: ", facing)
                move_list.append('F:' + str(distance1))
                facing = direction
                #print("facing: ", facing)

            #go East
            elif direction == "East":
                if facing == "North":
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:R')
                    else:
                        move_list.append('T:CITY:R')
                    #move_list.append('F:' + str(distance1))
                    #print("previously facing: ", facing)
                elif facing == "South":
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:L')
                    else:
                        move_list.append('T:CITY:L')
                    #move_list.append('F:' + str(distance1))
                    #print("previously facing: ", facing)
                elif facing == "East":
                    pass
                    #move_list.append('F:' + str(distance1))
                    #print("previously facing: ", facing)
                elif facing == "West": #180
                    if self.corners[index][0] < 14:
                        move_list.append('T:TURF:ROTATE')
                    else:
                        move_list.append('T:CITY:ROTATE')
                    #move_list.append('F:' + str(distance1))
                    #print("previously facing: ", facing)
                move_list.append('F:' + str(distance1))
                facing = direction
                #print("facing: ", facing)
            else:
                move_list.append('Stop')

        print(move_list)

    ########################################
    #convert encoder coord to virtual coord# 
    ########################################
    def gridCoord(self, x_cm, y_cm):
        dimensions = self.map.shape
        cvx = dimensions[1] - 1
        cvy = dimensions[0] - 1
        x = int(y_cm / 10.16)
        y = cvx - int(x_cm / 10.16)
        return (x,y)

#################################
#mutable grid of obstacle course#
#################################
class gridCourse():
    def __init__(self, gridmap):
        self.gridmap = numpy.array([
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #fifthloc: (1,19)
            [1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1], 

            [1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1], #sixthloc: (4,1), fourthloc: (4,22)
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],

            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],

            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],


            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,0,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #thirdloc: (13,1)
            [1,1,1, 1,1,1, 1,1,1, 1,0,0, 0,0,0, 0,0,0, 0,0,1, 1,1,1],

            [1,1,1, 1,1,1, 1,1,1, 1,0,0, 0,0,0, 0,0,0, 0,0,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #secondloc: (16,1)
            [1,1,1, 1,1,1, 1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,0,1, 1,1,1],

            [1,1,1, 1,1,1, 1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,0,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,1, 1,1,1, 1,1,1, 1,0,0, 0,0,1], #firstloc: (19,22), yellow: (19,1)
            [1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],

            [1,1,1, 1,0,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1],
            [1,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,1], #start: (22, 1), red: (22,22)
            [1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1, 1,1,1]])
    
    ##############
    #get the grid#
    ##############
    def getgridmap(self):
        return self.gridmap





