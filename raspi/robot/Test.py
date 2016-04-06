from Pathfinding import *
import time
import numpy

#########################################
#test class gridCourse and mutable array#
#########################################
def test2():
    #get grid class
    gridclass = gridCourse(None)

    #get grid
    getgrid = gridclass.getgridmap() 

    #get pathfinder class
    testing = PathFinder(getgrid)
    #start to first loc
    find1 = testing.findCorners((22,1), (19,22))
    moves1 = testing.actionList()
    print("start to first: ", moves1)

    testing = PathFinder(getgrid)
    #firstloc to red dropoff
    find1r = testing.findCorners((19,22),(22,22))
    moves1r = testing.actionList()
    print("first to red: ", moves1r)

    testing= PathFinder(getgrid)
    #firstloc to yellow dropoff
    find1y = testing.findCorners((19,22),(19,1))
    moves1y = testing.actionList()
    print("first to yellow: ", moves1y)

    #print(getgrid)

    #edit grid
    getgrid = gridclass.editgridmap((18,22),(17,22))
    testing = PathFinder(getgrid)
    #firstloc to secondloc
    find2 = testing.findCorners((19,22), (16,1))
    moves2 = testing.actionList()
    print("first to second: ", moves2)

    testing = PathFinder(getgrid)
    #red to secondloc
    findr2 = testing.findCorners((22,22),(16,1))
    movesr2 = testing.actionList()
    print("red to second: ", movesr2)

    testing = PathFinder(getgrid)
    #yellow to secondloc
    findy2 = testing.findCorners((19,1),(16,1))
    movesy2 = testing.actionList()
    print("yellow to second: ", movesy2)

    testing = PathFinder(getgrid)
    #secondloc to red
    find2r = testing.findCorners((16,1),(22,22))
    moves2r = testing.actionList()
    print("second to red: ", moves2r)

    testing = PathFinder(getgrid)
    #secondloc to yellow
    find2y = testing.findCorners((16,1),(19,1))
    moves2y = testing.actionList()
    print("second to yellow: ", moves2y)

    #print(getgrid)

    getgrid = gridclass.editgridmap((15,4),(14,4))
    testing = PathFinder(getgrid)

    #secondloc to thirdloc
    find3 = testing.findCorners((16,1),(13,1))
    moves3 = testing.actionList()
    print("second to third: ", moves3)

    testing = PathFinder(getgrid)
    #red to thirdloc
    findr3 = testing.findCorners((22,22),(13,1))
    movesr3 = testing.actionList()
    print("red to third: ", movesr3)

    testing = PathFinder(getgrid)
    #yellow to thirdloc
    findy3 = testing.findCorners((19,1),(13,1))
    movesy3 = testing.actionList()
    print("yellow to third: ", movesy3)

    testing = PathFinder(getgrid)
    #thirdloc to red
    find3r = testing.findCorners((13,1),(22,22))
    moves3r = testing.actionList()
    print("third to red: ", moves3r)

    testing = PathFinder(getgrid)
    #thirdloc to yellow
    find3y = testing.findCorners((13,1),(19,1))
    moves3y = testing.actionList()
    print("third to yellow: ", moves3y)

    testing = PathFinder(getgrid)
    #thirdloc to fourthloc
    find4 = testing.findCorners((13,1),(4,22))
    moves4 = testing.actionList()
    print("third to fourth: ", moves4)

    testing = PathFinder(getgrid)
    #red to fourthloc
    findr4 = testing.findCorners((22,22),(4,22))
    movesr4 = testing.actionList()
    print("red to fourth: ", movesr4)

    testing = PathFinder(getgrid)
    #yellow to fourthloc
    findy4 = testing.findCorners((19,1),(4,22))
    movesy4 = testing.actionList()
    print("yellow to fourth: ", movesy4)

    testing = PathFinder(getgrid)
    #fourth to red
    find4r = testing.findCorners((4,22),(22,22))
    moves4r = testing.actionList()
    print("fourth to red: ", moves4r)

    testing = PathFinder(getgrid)
    #fourth to yellow
    find4y  = testing.findCorners((4,22),(19,1))
    moves4y = testing.actionList()
    print("fourth to yellow: ", moves4y)

    testing = PathFinder(getgrid)
    #fourthloc to fifthloc
    find5 = testing.findCorners((4,22),(1,19))
    moves5 = testing.actionList()
    print("fourth to fifth: ", moves5)

    testing = PathFinder(getgrid)
    #red to fifthloc
    findr5 = testing.findCorners((22,22),(1,19))
    movesr5 = testing.actionList()
    print("red to fifth: ", movesr5)

    testing = PathFinder(getgrid)
    #yellow to fifthloc
    findy5 = testing.findCorners((19,1),(1,19))
    movesy5 = testing.actionList()
    print("yellow to fifth: ", movesy5)

    testing = PathFinder(getgrid)
    #fifth to red
    find5r = testing.findCorners((1,19),(22,22))
    moves5r = testing.actionList()
    print("fifth to red: ", moves5r)

    testing = PathFinder(getgrid)
    #fifth to yellow
    find5y = testing.findCorners((1,19),(19,1))
    moves5y = testing.actionList()
    print("fifth to yellow: ", moves5y)

    testing = PathFinder(getgrid)
    #fifthloc to sixthloc
    find6 = testing.findCorners((1,19),(4,1))
    moves6 = testing.actionList()
    print("fifth to sixth: ", moves6)

    testing = PathFinder(getgrid)
    #red to sixthloc
    findr6 = testing.findCorners((22,22),(4,1))
    movesr6 = testing.actionList()
    print("red to sixth: ", movesr6)

    testing = PathFinder(getgrid)
    #yellow to sixthloc
    findy6 = testing.findCorners((19,1),(4,1))
    movesy6 = testing.actionList()
    print("yellow to sixth: ", movesy6)

    testing = PathFinder(getgrid)
    #sixth to red
    find6r = testing.findCorners((4,1),(22,22))
    moves6r = testing.actionList()
    print("sixth to red: ", moves6r)

    testing = PathFinder(getgrid)
    #sixth to yellow
    find6y = testing.findCorners((4,1),(19,1))
    moves6y = testing.actionList()
    print("sixth to yellow: ", moves6y)

    testing = PathFinder(getgrid)
    #red to start
    findrstart = testing.findCorners((22,22),(22,1))
    movesrstart = testing.actionList()
    print("red to start: ", movesrstart)

    testing = PathFinder(getgrid)
    #yellow to start
    findystart = testing.findCorners((19,1),(22,1))
    movesystart = testing.actionList()
    print("yellow to start: ", movesystart)

    #print(getgrid)

    testing = PathFinder(getgrid)
    #obstacle location
    findobstacle = testing.findCorners((0,0),(19,22))
    movesobstacle = testing.actionList()
    print("obstacle to goal: ", movesobstacle)

    testing = PathFinder(getgrid)
    findobstacle1 = testing.findCorners((9,9),(13,1))
    movesobstacle1 = testing.actionList()
    print("obstacle1 to goal: ", movesobstacle1)

"""
#pseudocode for actual run
def runCourse():
    victimloc = [(19,22), (16,2), (13,2), (4,22), (1,19), (4,1)]
    dropoffs = {'red': (22,22), 'yellow': (19,2)}
    startpos = (22,1)

    #need a victim count

    gridclass = gridCourse(None)
    getgrid = gridclass.getgridmap()
    for goal in victimloc:
        testing = PathFinder(getgrid)
        find = testing.findCorners(startpos, goal)
        #path between unknown gap is False
        #reset pathfind to travel along the hallway where the gap may be
        if find == False:
            #edit grid for gap
            getgrid = gridclass.editgridmap((currentpos[0]+1, currentpos[1]),(currentpos[0]+2, currentpos[1]))
            testing = PathFinder(getgrid)
            findfromgap = testing.findCorners(currentpos, goal)
            movesfromgap = testing.actionList()
            print(movesfromgap)
        else:
            moves = testing.actionList()
            print(moves)
        startpos = goal

        #if victim found, go to red or yellow drop off zone
        if victim == True:
            if color == 'Y':
                goal = dropoffs.get('yellow')
                find = testing.findCorners(startpos, goal)
            else: #color == 'R':
                goal = dropoffs.get('red')
                find = testing.findCorners(startpos, goal)
            startpos = goal
            #decrement victim count
"""            


#timing
start = time.clock()
test2()
end = time.clock()
print(end-start)