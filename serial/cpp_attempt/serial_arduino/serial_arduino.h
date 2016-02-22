/////////////////////////////////////////////////////////////////////////
// serial_arduino.h
//
// header file for serial communication between raspberry pi and arduino
// these declarations are for the arduino due side of this module
//
// Author: Thomas Lavastida
// February 9, 2016
/////////////////////////////////////////////////////////////////////////

#ifndef SERIAL_ARDUINO_H
#define SERIAL_ARDUINO_H

#include "Arduino.h"


class serial_arduino
{
public:
    serial_arduino();
    serial_arduino(int br);

    void begin();
    void end();

    void send_data(char* send_buffer, int num_bytes);
    void recv_data(char* recv_buffer, int num_bytes);

private:
    int baud_rate;

};

#endif