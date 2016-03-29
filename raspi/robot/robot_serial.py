#!/usr/bin/env python3

import serial

class RobotSerial:
    # def __init__(self):
    #     self.portname = "/dev/ttyACM0"
    #     self.baud_rate = 115200
    #     self.timeout = 1
    #     self.port = serial.Serial(portname,baud_rate,timeout)

    def __init__(self,portname = "/dev/ttyACM0",baud_rate = 115200,timeout = 1):
        self.portname = portname
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.port = serial.Serial(portname,baud_rate,timeout=timeout)

    def __del__(self):
        self.port.close()

    def send(self,msg):
        if type(msg) is bytes:
            self.port.write(msg)
        elif type(msg) is str:
            self.port.write(msg.encode())
        else:
            raise TypeError('msg needs to be of type bytes or str')

    def recv(self):
        return self.port.readline()

    def name(self):
        return self.port.name



