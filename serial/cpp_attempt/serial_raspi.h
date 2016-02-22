/////////////////////////////////////////////////////////////////////////
// serial_raspi.h
//
// header file for serial communication between raspberry pi and arduino
// these declarations are for the raspberry pi side of this module
//
// Author: Thomas Lavastida
// February 8, 2016
/////////////////////////////////////////////////////////////////////////

#ifndef SERIAL_RASPI_H
#define SERIAL_RASPI_H

//includes
#include <termios.h>
#include <iostream>

class serial_raspi
{
//list the interface for the class first
public:
	//constructor
	serial_raspi();
	serial_raspi(std::string port, int baud);

	void begin();
	void end();

	void send_data(char* send_buffer, int num_bytes);  //consider changing to uint8_t
	void recv_data(char* recv_buffer, int num_bytes);

//then the private data members
private:
	std::string port_string;
	int port;
	int baud_rate;

	struct termios termios_options;

//private helper function
private:
    void establish_connection();

};

#endif