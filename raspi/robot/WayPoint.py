import numpy
import time

waypoint = [
     #0         #1          #2         #3         #4         #5         #6         #7         #8         #9         #10        #11        #12        #13        #14        #15        #16        #17        #18        #19        #20        #21        #22        #23
    [(0,''),    (1,'E'),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #0
    
    [(1,'W'),   (0,''),     (1,'N'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (6,'E'),   (0,'')], #1
    
    [(0,''),    (1,'S'),    (0,''),    (2,'E'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'W'),   (0,''),    (0,'')], #2
    
    [(0,''),    (0,''),     (2,'W'),   (0,''),    (3,'E'),   (0,''),    (0,''),    (1,'N'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #3
    
    [(0,''),    (0,''),     (0,''),    (3,'W'),   (0,''),    (1,'E'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #4 
    
    [(0,''),    (0,''),     (0,''),    (0,''),    (1,'W'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #5

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (3,'W'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'N'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #6
    
    [(0,''),    (0,''),     (0,''),    (1,'S'),   (0,''),    (0,''),    (3,'E'),   (0,''),    (2,'W'),   (0,''),    (1,'N'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #7

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (2,'E'),   (0,''),    (1,'W'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #8

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'E'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #9

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'S'),   (0,''),    (0,''),    (0,''),    (2,'W'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #10

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (2,'E'),   (0,''),    (1,'W'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #11

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'E'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #12

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (1,'S'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'E'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #13

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'W'),   (0,''),    (2,'N'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #14

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (2,'S'),   (0,''),    (1,'N'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #15

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'S'),   (0,''),    (1,'N'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #16

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'S'),   (0,''),    (1,'W'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #17

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'E'),   (0,''),    (6,'W'),   (0,''),    (0,''),    (0,''),    (0,'')], #18

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (6,'E'),   (0,''),    (1,'S'),   (0,''),    (0,''),    (0,'')], #19

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'N'),   (0,''),    (0,''),    (0,''),    (0,'')], #20

    [(0,''),    (0,''),     (1,'E'),   (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,'')], #21

    [(0,''),    (5,'W'),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'E')],#22

    [(0,''),    (0,''),     (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (0,''),    (1,'W'),   (0,'')]]#23
     #0         #1          #2         #3         #4         #5         #6         #7         #8         #9         #10        #11        #12        #13        #14        #15        #16        #17        #18        #19        #20        #21        #22        #23


print(waypoint[0])