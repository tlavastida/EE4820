#!/usr/bin/env python3

#robot_control.py
#main control loop for the robot

#Thomas Lavastida
#March 9, 2016

import robot_serial
#import numpy
import Pathfinding
import color_filter
import picamera
import picamera.array
import cv2
import time

class Robot:
    #constructor
    def __init__(self):

        ### Serial Stuff ###
        self.portname = '/dev/ttyACM0'
        self.baud = 250000
        self.timeout = 2
        self.ser = robot_serial.RobotSerial(self.portname,self.baud,self.timeout)
        ####################

        ### There are four total victims ###
        self.total_victims = 4
        self.found_victims = 0
        ####################

        ### State variables ###
        self.grid_x = 0
        self.grid_y = 0
        #insert variable/s for encoder measurements maybe?
        self.dx = 1
        self.dy = 0

        ### Camera Stuff ###
        self.cam = picamera.PiCamera()
        
    def print_state(self):
        print('rescued victims: ' + str(self.found_victims))
        print('position: (' + str(self.grid_x) + ',' + str(self.grid_y))
        print('direction: (' + str(self.dx) + ',' + str(self.dy))
        
        
    #updates grid positions in the map using an encoder reading
    #def update_grid_position(self,encoder1,encoder2):
    #    avg = (encoder1+encoder2)//2 #integer division for now, maybe try floating point plus round later
    #    self.grid_x += self.dx*avg   #FIX LATER - NEED CORRECT CONVERSION FACTOR TO GRID SPACE
    #    self.grid_y += self.dy*avg   #FIX LATER - NEED CORRECT CONVERSION FACTOR TO GRID SPACE

    #abstracting the pattern from the other functions I wrote:
    def exec_cmd(self,cmd_str):
        self.ser.send(cmd_str)
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        return msg #the callee decides how to process the info in the message


    #functions I already wrote, all had a similar pattern
    def forward(self,distance):
        print('G'+str(distance))
        self.ser.send('G'+str(distance))
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        return msg #PLACEHOLDER
        
    #tells arduino to turn left
    def turn_left(self,num_turns):
        self.ser.send('T'+str(num_turns)+'L')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        self.dx = -1*self.dy
        self.dy = self.dx
        return msg #PLACEHOLDER

    #tells arduino to turn right
    def turn_right(self,num_turns):
        self.ser.send('T'+str(num_turns)+'R')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        self.dx = self.dy
        self.dy = -1*self.dx
        return msg #PLACEHOLDER

    #tells arduino to acquire a target victim and pickup
    def pickup(self):
        self.ser.send('P')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        #update state?

        return msg #PLACEHOLDER

    def lower_gripper(self):
        self.ser.send('MD')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        return msg

    def raise_gripper(self):
        self.ser.send('MU')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        return msg

    def open_grip(self):
        self.ser.send('MO')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        return msg
    
    def close_grip(self):
        self.ser.send('MC')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        return msg

    def direction(self):
        if self.dx == 1 and self.dy == 0:
            return 'East'
        elif self.dx == -1 and self.dy == 0:
            return 'West'
        elif self.dx == 0 and self.dy == 1:
            return 'North'
        elif self.dx == 0 and self.dy == -1:
            return 'South'
        else:
            return None

    def detectVictim(self):
        with picamera.array.PiRGBArray(self.cam) as stream:
            self.cam.capture( stream, format='bgr', resize=(320,240) )
            # At this point the image is available as stream.array
            image = stream.array
            return color_filter.detectVictim(image)
        #self.cam.capture(stream, format='bgr')#,resize=(320,240))
        #img = self.stream.array
        #return color_filter.detectVictim(img)
        

def turn_test():
    sara = Robot()
    
    sara.turn_left()
    time.sleep(2.0)
    sara.turn_right()
        
    #signify end of test
    sara.close_grip()
    time.sleep(1.0)
    sara.open_grip()
    #done


def forward_test():
    sara = Robot()

    sara.forward(300)

    #signify end of test
    sara.close_grip()
    time.sleep(1.0)
    sara.open_grip()
    #done

def camera_test():
    sara = Robot()



def main_loop():

    sara = Robot()
    grid = Pathfinding.gridCourse(None)
    pathfind = Pathfinding.PathFinder(grid.gridmap)

    targetList = [(19,21), (16,2), (13,2), (5,22), (1,20), (3,1)]
    victimNodes = { targetList[0]:(19,22), targetList[1]:(16,1), targetList[2]:(13,1), targetList[3]:(4,22), targetList[4]:(1,19), targetList[5]:(4,1) }


    startpos = (22,1)
    sara.grid_x = 22
    sara.grid_y = 1

    for target in targetList:    #maybe use pop
        pathfind.findCorners( (sara.grid_x,sara.grid_y), target)
        actions = pathfind.actionList( sara.direction() )

        #carry out all the moves
        for move in actions:
            words = move.split(':')

            if words[0] == 'F':
                pass #move forward

            elif words[0] == 'T':
                pass #turns ...
                
            else:
                pass #default case

            time.sleep(1.5)

                
        #turn around to check for victim
        sara.about_face()
        time.sleep(1.5)

        #check for victim
        if sara.detectVictim():
            sara.pickup()
            #find return path
            sara.grid_x = target[0] - sara.dx
            sara.grid_y = target[1] - sara.dy


    
def test_run():
    sara = Robot()
    time.sleep(2)
    #grid = Pathfinding.gridCourse(None)
    pathfind = Pathfinding.PathFinder()  #grid.gridmap)

    sara.grid_x = 22
    sara.grid_y = 1

    target = (19,22)

    pathfind.findCorners( (sara.grid_x,sara.grid_y), target )
    actions = pathfind.actionList( sara.direction() )

    print('Computed path, starting route...')

    
    for move in actions:
        print(move)
        words = move.split(':')
        if words[0] == 'F':
            grid_dist = int(words[1])
            cms = (1016*grid_dist)//1000   #this bitch right here
            print(cms)
            msg = sara.forward(cms)
        elif words[0] == 'T' and words[1] == 'R':
            msg = sara.turn_right(1)
        elif words[0] == 'T' and words[1] == 'L':
            msg = sara.turn_left(1)
        else:
            msg = 'Not a valid move?'

        print(msg)
        time.sleep(1.5)
        

    print('YAY..... maybe?')
    
    
def interactive_main_loop():
    
    #my most favorite line of code ever -- Thomas
    sara = Robot()

    #cmd_str = ''
    text = input('>>> ')
    while text != 'exit':
        words = text.split()
        cmd = words[0]
        if cmd == 'go':
            if len(words) < 2:
                distance = 30
            else:
                distance = int(words[1])
            msg = sara.forward(distance)
            print(msg)
            #print('going forward ' + str(distance) + ' centimeters')
        elif cmd == 'left':
            if len(words) < 2:
                num_words = 1
            else:
                num_words = int(words[1])
            msg = sara.turn_left(num_words)
            print(msg)
            #print('turning left')
        elif cmd == 'right':
            if len(words) < 2:
                num_turns = 1
            else:
                num_turns = int(words[1])
            msg = sara.turn_right(num_turns)
            print(msg)
            #print('turning right')
        elif cmd == 'lower':
            msg = sara.lower_gripper()
            print(msg)
        elif cmd == 'raise':
            msg = sara.raise_gripper()
            print(msg)
        elif cmd == 'close':
            msg = sara.close_grip()
            print(msg)
        elif cmd == 'open':
            msg = sara.open_grip()
            print(msg)
        elif cmd == 'pickup':
            msg = sara.pickup()
            print(msg)
        elif cmd == 'help':
            print('go <distance> --- move forward <distance> centimeters')
            print('left --- turn 90 degrees left')
            print('right --- turn 90 degrees right')
            print('exit --- terminate this program')
            print('help --- view list of commands')
        else:
            print('Not a valid command, type help for a list of commands')

        text = input('>>> ')


        
if __name__ == '__main__':
    #interactive_main_loop()
    #test_run()
    #forward_test()
    print('Script is running ... ')    
    
