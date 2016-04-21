#!/usr/bin/env python3

#robot_control.py
#main control loop for the robot

#Thomas Lavastida
#March 9, 2016

import robot_serial
#import numpy
import color_filter
import WayPoint

import picamera
import picamera.array
import cv2
import time

class Robot:
    #constructor
    def __init__(self):

        ### Serial Stuff ###
        self.portname = '/dev/ttyACM0'
        self.baud = 115200  # 250000
        self.timeout = 2
        self.ser = robot_serial.RobotSerial(self.portname,self.baud,self.timeout)
        ####################

        ### There are four total victims ###
        self.total_victims = 4
        self.found_victims = 0
        ####################

        ### State variables ###
        self.direction = 'E'

        ### Camera Stuff ###
        #self.cam = picamera.PiCamera()
        

    #abstracting the pattern from the other functions I wrote:
    def exec_cmd(self,cmd_str):
        self.ser.send(cmd_str)
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        return msg #the callee decides how to process the info in the message


    #functions I already wrote, all had a similar pattern
    def forward(self,distance):
        print('G'+str(distance)+'Z')
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
        
        #update direction
        if self.direction == 'N':
            self.direction = 'W'

        elif self.direction == 'W':
            self.direction = 'S'

        elif self.direction == 'E':
            self.direction = 'N'

        elif self.direction == 'S':
            self.direction = 'E'

        return msg

    #tells arduino to turn right
    def turn_right(self,num_turns):
        self.ser.send('T'+str(num_turns)+'R')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        if self.direction == 'N':
            self.direction = 'E'

        elif self.direction == 'W':
            self.direction = 'N'

        elif self.direction == 'E':
            self.direction = 'S'

        elif self.direction == 'S':
            self.direction = 'W'

        return msg

    #tells arduino to make a 180 degree turn
    def about_face(self):
        self.ser.send('A0Z')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        if self.direction == 'N':
            self.direction = 'S'
        elif self.direction == 'S':
            self.direction = 'N'
        elif self.direction == 'W':
            self.direction = 'E'
        elif self.direction == 'E':
            self.direction = 'W'

        return msg

    #tells arduino to acquire a target victim and pickup
    def pickup(self):
        self.ser.send('P0Z')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        #update state?

        return msg #PLACEHOLDER

    def dropoff(self):
        self.ser.send('D0Z')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        #update state?
        return msg #PLACEHOLDER        


    def lower_gripper(self):
        self.ser.send('M'+ str(0) + 'D')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        return msg

    def raise_gripper(self):
        self.ser.send('M' + str(0) + 'U')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        return msg

    def open_grip(self):
        self.ser.send('M'+ str(0) +'O')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()
        return msg
    
    def close_grip(self):
        self.ser.send('M' + str(0) + 'C')
        while self.ser.available() <= 0:
            pass
        msg = self.ser.recv()

        return msg

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
    graph = WayPoint.WayPoint()






    
def test_run():

    sara = Robot()
    graph = WayPoint.WayPoint()


    start = 0
    target1 = 4
    #target2 = 23

    path = graph.search(start,target1)

    current = start
    for i in path[1:]:
        (distance,direction) = graph.getData(current,i)

        if direction == sara.direction:
            sara.forward(distance)
        else:
            if direction == 'N':
                if sara.direction == 'S':
                    sara.about_face()
                elif sara.direction == 'E':
                    sara.turn_left(1)
                elif sara.direction == 'W':
                    sara.turn_right(1)


            elif direction == 'W':
                if sara.direction == 'N':
                    sara.turn_left(1)
                elif sara.direction == 'S':
                    sara.turn_right(1)
                elif sara.direction == 'E':
                    sara.about_face()


            elif direction == 'E':
                if sara.direction == 'W':
                    sara.about_face()
                elif sara.direction == 'N':
                    sara.turn_right(1)
                elif sara.direction == 'S':
                    sara.turn_left(1)

            elif direction == 'S':
                if sara.direction == 'N':
                    sara.about_face()
                elif sara.direction == 'W':
                    sara.turn_left(1)
                elif sara.direction == 'E':
                    sara.turn_right(1)

            #now direction == sara.direction
            sara.forward(distance)

        time.sleep(1.0)
        current = i

    #(success,color) = sara.detectVictim()
    #reached target location
    sara.pickup()

    #if color == 'RED':
    #    sara.open_grip()



    print('Yay .... ?')




    
    
def interactive_main_loop():
    
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
    test_run()
    #forward_test()
    #print('Script is running ... ')    
    
