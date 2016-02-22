/////////////////////////////////////////////////////////////////////////
// test_serial.cpp
//
// unit test for the serial module.
// sends a number n to the arduino, then a '+' or '*'. This is followed 
// by a sequence of n random numbers.  The arduino should compute the
// result then send it back to have the answer verified. 
//
// Author: Thomas Lavastida
// February 9, 2016
/////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string.h>
#include "serial_raspi.h"

int main(int argc, char** argv)
{
    //init
    serial_raspi serial("/dev/ttyACM0",115200);
    serial.begin();


    const int range = 200;
    srand(time(NULL));

    //put test code here
    int x,y,z;  //z == x + y is the test
    int op;
    char send_buf[64];
    char recv_buf[64];
    int n;
    int result;

    int num_passed = 0;

    std::cout << "beginning tests..." << std::endl << std::endl;

    for(int i = 0; i < 100; ++i)  //do 100 tests
    {

        x = rand() % range + 1;
        y = rand() % range + 1;
        op = rand() % 2;

        if(op) 
        {
            z = x + y;
            n = sprintf(send_buf,"+,%d,%d",x,y);
        }
        else
        {
            z = x*y;
            n = sprintf(send_buf,"*,%d,%d",x,y);  
        }
        std::cout << "sending..." << std::endl;
        serial.send_data(send_buf,n);
        std::cout << "receiving..." << std::endl;
        serial.recv_data(recv_buf,16);

        //result = *(int*)(recv_buf);
        memcpy(&result,recv_buf,sizeof(result));

        std::cout << "sent: " << send_buf << std::endl;
        std::cout << "received: " << result << std::endl;
        std::cout << "test: " << x << (op ? " + " : " * ") << y << " = " << z << std::endl;
        if(z == result)
        {
            std::cout << "test passed";
            num_passed++;
        }
        else
            std::cout << "test failed";
        std::cout << std::endl << std::endl;

    }

    std::cout << "number of tests passed: " << num_passed << "/100" << std::endl;


    //cleanup
    serial.end();
    return 0;
}