#!/usr/bin/env python3

import serial

class RobotSerial:
    # def __init__(self):
    #     self.portname = "/dev/ttyACM0"
    #     self.baud_rate = 115200
    #     self.timeout = 1
    #     self.port = serial.Serial(portname,baud_rate,timeout)

    #constructor
    def __init__(self,portname = "/dev/ttyACM0",baud_rate = 115200,timeout = 1):
        self.portname = portname
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.port = serial.Serial(portname,baud_rate,timeout=timeout)

    #destructor
    def __del__(self):
        self.port.close()

    #sends data to device
    def send(self,msg):
        if type(msg) is bytes:
            self.port.write(msg)
        elif type(msg) is str:
            self.port.write(msg.encode())
        else:
            raise TypeError('msg needs to be of type bytes or str')

    #receives data from device
    def recv(self):
        return self.port.readline()

    #returns number of bytes in recv buffer
    def available(self):
        return self.port.in_waiting()
    
    def name(self):
        return self.port.name



