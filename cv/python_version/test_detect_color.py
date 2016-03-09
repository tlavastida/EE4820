#!/usr/bin/env python3

#test_detect_color.py
#March 8, 2016
#Thomas Lavastida

#unit test for color detection procedures

import cv2
import numpy as np
import time

#unit under test
import color_filter


if __name__ == '__main__':

    prefix = './imgs/'
    suffix = '.jpg'
    red = 'red'
    yel = 'yellow'

    num_tests = 10

    num_passed = 0

    start = time.time()
    
    for i in range(num_tests):
        file_red = prefix + red + str(i) + suffix
        img_red = cv2.resize(cv2.imread(file_red), (320,240))

        if color_filter.detectColor(img_red) == 'RED':
            print(file_red + ': success')
            num_passed += 1
        else:
            print(file_red + ': failure')

        file_yel = prefix + yel + str(i) + suffix
        img_yel = cv2.resize(cv2.imread(file_yel), (320,240))

        if color_filter.detectColor(img_yel) == 'YELLOW':
            print(file_yel + ': success')
            num_passed += 1
        else:
            print(file_yel + ': failure')

    end = time.time()
            
    print('\n\n===============================================')
    print('Report:')
    print('Number of tests: ' + str(2*num_tests))
    print('Elapsed time: ' + str(end-start) + ' seconds')
    print('Passed: ' + str(num_passed) + '/' + str(2*num_tests))
    print('===============================================')
        
