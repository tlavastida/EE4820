//serial.cpp

#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>

int main(int argc, char const *argv[])
{
    
    char* portname = "/dev/ttyACM0";
    char buf[256];

    int file_descriptor = open(portname, O_RDWR | O_NOCTTY);

    struct termios termios_options;

    //get current settings
    tcgetattr(file_descriptor,&termios_options);
    //now set custom settings
    cfsetispeed(&termios_options,B9600);//baud rate
    cfsetospeed(&termios_options,B9600);

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
    tcsetattr(file_descriptor, TCSANOW, &termios_options);

    usleep(1000*1000);
    tcflush(file_descriptor, TCIFLUSH);

    char c;
    while( c != 'q')
    {

        int n = read(file_descriptor,buf,128);
        buf[n>0?n:0] = 0;
        printf("%i bytes read from device...\n", n);
        printf("buffer contents: %s\n", buf);

        std::cout << "quit? >> ";
        std::cin >> c;
    }

    return 0;
}
