#!/usr/bin/env python3

#robot_control.py
#main control loop for the robot

#Thomas Lavastida
#March 9, 2016

import robot_serial
#import numpy


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

    def print_state(self):
        print('rescued victims: ' + str(self.found_victims))
        print('position: (' + str(self.grid_x) + ',' + str(self.grid_y))
        print('direction: (' + str(self.dx) + ',' + str(self.dy))
        
        
    #updates grid positions in the map using an encoder reading
    def update_grid_position(self,encoder1,encoder2):
        avg = (encoder1+encoder2)//2 #integer division for now, maybe try floating point plus round later
        self.grid_x += self.dx*avg   #FIX LATER - NEED CORRECT CONVERSION FACTOR TO GRID SPACE
        self.grid_y += self.dy*avg   #FIX LATER - NEED CORRECT CONVERSION FACTOR TO GRID SPACE

    #abstracting the pattern from the other functions I wrote:
    def exec_cmd(self,cmd_str):
        self.ser.send(cmd_str)
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        return msg #the callee decides how to process the info in the message


    #functions I already wrote, all had a similar pattern
    def forward(self,distance):
        self.ser.send('G'+str(distance))
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        return msg #PLACEHOLDER
        
    #tells arduino to turn left
    def turn_left(self):
        self.ser.send('T1L')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        self.dx = -1*self.dy
        self.dy = self.dx
        return msg #PLACEHOLDER

    #tells arduino to turn right
    def turn_right(self):
        self.ser.send('T1R')
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
            msg = sara.forward('G'+str(distance))
            print(msg)
            #print('going forward ' + str(distance) + ' centimeters')
        elif cmd == 'left':
            msg = sara.turn_left()
            print(msg)
            #print('turning left')
        elif cmd == 'right':
            msg = sara.turn_right()
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
    interactive_main_loop()
        
    
    
