#!/usr/bin/env python3

#serial_test.py

import serial
from time import sleep

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0',250000, timeout = 1)
    print(ser.name )
    print('\n\n')


    c = input()
    while c != 'q':
        ser.write( (c+'\n').encode() )

        ret = ser.readline()
        print(ret.decode()) 
        c = input()

    ser.close()
