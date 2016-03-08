#!/usr/bin/env python3

import cv2
import numpy as np

img = cv2.imread('objects.jpg')

edges = cv2.Canny(img,100,200)

cv2.imwrite('output.jpg',edges)
