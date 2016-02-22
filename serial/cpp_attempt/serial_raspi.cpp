/////////////////////////////////////////////////////////////////////////
// serial_raspi.cpp
//
// source file for serial communication between raspberry pi and arduino
// this code is intended for the raspberry pi side of this module
//
// Author: Thomas Lavastida
// February 8, 2016
/////////////////////////////////////////////////////////////////////////

//includes
#include "serial_raspi.h"

#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>


serial_raspi::serial_raspi()
{
	port_string = "/dev/ttyACM0";
	baud_rate = 9600;
}

serial_raspi::serial_raspi(std::string port, int baud)
{
	port_string = port;
	baud_rate = baud;
}

void serial_raspi::establish_connection()
{
    char arr[6] = {'s','e','t','u','p','\0'};
    write(port,arr,6);
}

void serial_raspi::begin()
{
	port = open(port_string.c_str(), O_RDWR | O_NOCTTY);

	if(!port)
	{
		exit(-1);
		//FATAL ERROR
	}

	//get settings
	tcgetattr(port,&termios_options);

	//replace settings
	speed_t speed;
	switch(baud_rate)
	{
		case 9600:	
			speed = B9600;
			break;
		case 115200:
			speed = B115200;
			break;
		default:
			speed = B9600;
			break;
	}
	cfsetispeed(&termios_options,speed);
	cfsetospeed(&termios_options,speed);

	//8 bits, no parity, no stop bit
    termios_options.c_cflag &= ~PARENB;
    termios_options.c_cflag &= ~CSTOPB;
    termios_options.c_cflag &= ~CSIZE;
    termios_options.c_cflag |= CS8;

    termios_options.c_cflag &= ~CRTSCTS; //no hardware flow control
    termios_options.c_cflag |= CREAD | CLOCAL;

    termios_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termios_options.c_oflag &= ~OPOST;

    termios_options.c_cc[VMIN] = 12;
    termios_options.c_cc[VTIME] = 0;

    //activate the settings;
    tcsetattr(port, TCSANOW, &termios_options);

    usleep(1000*1000);
    tcflush(port, TCIFLUSH);

    //ready to go now
    //establish_connection();

}

void serial_raspi::end()
{
	close(port);
}

void serial_raspi::send_data(char* send_buffer, int num_bytes)
{
    int n;
    int bytes_left = num_bytes;

    do {
        n = write(port,send_buffer,bytes_left);
        bytes_left -= n;
        send_buffer += n; //move the pointer up by n bytes to find unsent data
    } while(bytes_left > 0);

}

void serial_raspi::recv_data(char* recv_buffer, int num_bytes)
{
    
    int n;
    int bytes_left = num_bytes;

    do {
        std::cout << "bytes_left: " << bytes_left << std::endl;
        n = read(port,recv_buffer,bytes_left);
        bytes_left -= n;
        recv_buffer += n;
    } while(bytes_left > 0);

    //read(port,recv_buffer,num_bytes);
}

