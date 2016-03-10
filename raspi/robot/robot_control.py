#!/usr/bin/env python3

#robot_control.py
#main control loop for the robot

#Thomas Lavastida
#March 9, 2016

import numpy


#There are four victims we need to rescue
total_victims = 4


def main_loop():
    #initialization
    num_rescued = 0
    current_pos = np.array([0,0])
    
    #other initialiation....

    while num_rescued != total_victims:

        #get a path
        path = getPath()

        

        
        
    
    
