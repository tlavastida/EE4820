
#!/usr/bin/env python3

#color_filter.py
#March 3, 2016
#Thomas Lavastida
#Steve Surcouf

import cv2
import numpy as np

def brighten(img):
    alpha = 1.5
    beta = -10 
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
    lower = np.array([17,5,87],dtype = 'uint8')
    upper = np.array([75,75,255],dtype = 'uint8')
    mask = cv2.inRange(img,lower,upper)
    out = cv2.bitwise_and(img,img,mask = mask)
    #return out
    return (mask,out)

def filterYellow(img):
    lower = np.array([10,120,120],dtype = 'uint8')
    upper = np.array([75,220,220],dtype = 'uint8')
    mask = cv2.inRange(img,lower,upper)
    out = cv2.bitwise_and(img,img,mask = mask)
    #return out
    return (mask,out)

def detectColor(img):
    bright = brighten(img)

    (red,i1) = filterRed(bright)
    (yellow,i2) = filterYellow(bright)

    #totalRed = red.sum(axis=2).sum(axis=1).sum(axis=0)
    #totalYellow = yellow.sum(axis=2).sum(axis=1).sum(axis=0)

    totalRed = red.sum(axis=1).sum(axis=0)
    totalYellow = yellow.sum(axis=1).sum(axis=0)
    
    if totalRed > totalYellow:
        return 'RED'
    else:
        return 'YELLOW'

def detectVictim(img):
    threshold = 100
    bright = img #brighten(img)
    (red,i1) = filterRed(bright)
    (yellow,i2) = filterYellow(bright)

    totalRed = red.sum(axis=1).sum(axis=0)
    totalYellow = yellow.sum(axis=1).sum(axis=0)

    print('Red Score = ' + str(totalRed))
    print('Yellow Score = ' + str(totalYellow))
    
    if totalRed >= threshold or totalYellow >= threshold:
        return True
    else:
        return False
    
if __name__ == '__main__':

    #file1 = 'redvictim.jpg'
    #file2 = 'yellowvictim.jpg'

    file1 = './imgs/red8.jpg'
    #file2 = './imgs/red5.jpg'
    file2 = 'test.jpg'
    
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

    
    (red1,i1) = filterRed(bright1)
    (red2,i2) = filterRed(bright2)
    cv2.imshow('red components: ' + file1,red1)
    cv2.imshow('red components: ' + file2,red2)
    cv2.waitKey(0)

    (yellow1,i1) = filterYellow(bright1)
    (yellow2,i2) = filterYellow(bright2)
    cv2.imshow('yellow components: ' + file1,yellow1)
    cv2.imshow('yellow components: ' + file2,yellow2)
    cv2.waitKey(0)

    print(file1 + ' is recognized as ' + detectColor(img1))
    print(file2 + ' is recognized as ' + detectColor(img2))
    
    cv2.destroyAllWindows()

