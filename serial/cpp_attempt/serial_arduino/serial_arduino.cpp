/////////////////////////////////////////////////////////////////////////
// serial_arduino.cpp
//
// source file for serial communication between raspberry pi and arduino
// this code is intended for the arduino due side of this module
//
// Author: Thomas Lavastida
// February 9, 2016
/////////////////////////////////////////////////////////////////////////

#include "serial_arduino.h"

serial_arduino::serial_arduino()
{
    baud_rate = 9600;
}

serial_arduino::serial_arduino(int br)
{
    baud_rate = br;
}

void serial_arduino::begin()
{
    Serial.begin();
}

void serial_arduino::end()
{
    Serial.end();
}

void serial_arduino::send_data(char* send_buffer, int num_bytes)
{
    int num_sent = 0;
    int n; //current amt
    do {
        n = Serial.

    } while(num_sent < num_bytes)
}

void serial_arduino::recv_data(char* recv_buffer, int num_bytes)
{

}