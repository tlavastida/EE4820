#!/usr/bin/env python3

#color_filter.py
#March 3, 2016
#Thomas Lavastida
#Steve Surcouf

import cv2
import numpy as np

def brighten(img):
    alpha = 1.5
    beta = 30 
    h,w,c = img.shape
    zero = np.zeros((h,w,3), np.uint8)
    out = cv2.addWeighted(img,alpha,zero,0,beta)
    return out

def equalize(img):
    h,w,c = img.shape
    b,g,r = cv2.split(img)

    b = cv2.equalizeHist(b)
    g = cv2.equalizeHist(g)
    r = cv2.equalizeHist(r)

    out = cv2.merge((b,g,r))
    return out 

def filterRed(img):
    lower = np.array([17,15,90],dtype = 'uint8')
    upper = np.array([70,70,255],dtype = 'uint8')
    mask = cv2.inRange(img,lower,upper)
    out = cv2.bitwise_and(img,img,mask = mask)
    return out

def filterYellow(img):
    lower = np.array([25,146,190],dtype = 'uint8')
    upper = np.array([62,200,250],dtype = 'uint8')
    mask = cv2.inRange(img,lower,upper)
    out = cv2.bitwise_and(img,img,mask = mask)
    return out

def detectColor(img):
    bright = brighten(img)

    red = filterRed(bright)
    yellow = filterYellow(bright)

    totalRed = red.sum(axis=2).sum(axis=1).sum(axis=0)
    totalYellow = yellow.sum(axis=2).sum(axis=1).sum(axis=0)

    if totalRed > totalYellow:
        return 'RED'
    else:
        return 'YELLOW'
    
if __name__ == '__main__':

    file1 = 'redvictim.jpg'
    file2 = 'yellowvictim.jpg'

    img1 = cv2.imread(file1)
    img1 = cv2.resize(img1,(320,240))

    img2 = cv2.imread(file2)
    img2 = cv2.resize(img2,(320,240))
    
    cv2.imshow(file1,img1)
    cv2.imshow(file2,img2)
    cv2.waitKey(0)

    bright1 = brighten(img1)
    bright2 = brighten(img2)
    cv2.imshow(file1 + 'brightened',bright1)
    cv2.imshow(file2 + 'brightened',bright2)
    cv2.waitKey(0)

    
    red1 = filterRed(bright1)
    red2 = filterRed(bright2)
    cv2.imshow('red components: ' + file1,red1)
    cv2.imshow('red components: ' + file2,red2)
    cv2.waitKey(0)

    yellow1 = filterYellow(bright1)
    yellow2 = filterYellow(bright2)
    cv2.imshow('yellow components: ' + file1,yellow1)
    cv2.imshow('yellow components: ' + file2,yellow2)
    cv2.waitKey(0)

    print(file1 + ' is recognized as ' + detectColor(img1))
    print(file2 + ' is recognized as ' + detectColor(img2))
    
    cv2.destroyAllWindows()

