#!/usr/bin/env python3

#robot_control.py
#main control loop for the robot

#Thomas Lavastida
#March 9, 2016

import robot_serial
import numpy


class Robot:
    #constructor
    def __init__(self):

        ### Serial Stuff ###
        self.portname = "\dev\ttyACM0"
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

    #updates grid positions in the map using an encoder reading
    def update_grid_position(self,encoder1,encoder2):
        avg = (encoder1+encoder2)//2 #integer division for now, maybe try floating point plus round later
        self.grid_x += self.dx*avg   #FIX LATER - NEED CORRECT CONVERSION FACTOR TO GRID SPACE
        self.grid_y += self.dy*avg   #FIX LATER - NEED CORRECT CONVERSION FACTOR TO GRID SPACE

    def forward(self,distance):
        self.ser.send('G'+str(distance))
        while self.ser.available() <= 0:

        
        msg = self.ser.recv()
        return msg #PLACEHOLDER
        
    #tells arduino to turn left
    def turn_left(self):
        self.ser.send('T1L')
        while self.ser.available() <= 0:

        msg = self.ser.recv()
        self.dx = -1*dy
        self.dy = dx
        return msg #PLACEHOLDER

    #tells arduino to turn right
    def turn_right(self):
        self.ser.send('T1R')
        while self.ser.available() <= 0:

        msg = self.ser.recv()
        self.dx = dy
        self.dy = -1*dx
        return msg #PLACEHOLDER



def main_loop():
    
    #my most favorite line of code ever -- Thomas
    sara = Robot()
    
    text = input()
    while text != 'exit':
        words = text.split()
        cmd = words[0]
        if cmd == 'go':
            if not words[1]:
                distance = 30
            else:
                distance = int(words[1])
            msg = sara.forward(distance)
            print(msg)
        elif cmd == 'left':
            msg = sara.turn_left()
            print(msg)
        elif cmd == 'right':
            msg = sara.turn_right()
            print(msg)
        else:
            print('Not a valid command')

        text = input()


        
if __name__ == '__main__':
    mainloop()
        
    
    
