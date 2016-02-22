/////////////////////////////////////////////////////////////////////////
// test_echo.cpp
//
// unit test for the serial module.
// sends a buffer of text to the arduino, which the arduino then echoes
// back.
//
// Author: Thomas Lavastida
// February 9, 2016
/////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string.h>
#include <cstdio>
#include "serial_raspi.h"


int main(int argc, char const *argv[])
{
    
    serial_raspi serial("/dev/ttyACM0",115200);
    serial.begin();

    char buf[128];
    int n;

    while(true) {
        printf(">>> ");
        scanf("%s\n",buf);
        printf("\n");
        n = strlen(buf);
        
        serial.send_data(buf,n);
        serial.recv_data(buf,n);

        printf("received: %s\n\n",buf);
    }



    serial.end();
    return 0;
}