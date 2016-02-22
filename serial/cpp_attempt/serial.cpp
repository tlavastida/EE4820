//serial.cpp

#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int main(int argc, char const *argv[])
{
    
    char* portname = "/dev/ttyACM0";
    char buf[1024];

    int file_descriptor = open(portname, O_RDWR | O_NOCTTY);

    struct termios termios_options;

    //get current settings
    tcgetattr(file_descriptor,&termios_options);
    //now set custom settings
    //cfsetispeed(&termios_options,B9600);//baud rate
    //cfsetospeed(&termios_options,B9600);

    cfsetispeed(&termios_options,B115200);//baud rate
    cfsetospeed(&termios_options,B115200);

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
    int n;
    while( c != 'q')
    {

        std::cout << "quit? >> ";
        std::cin >> c;
        write(file_descriptor,&c,1);
        n = read(file_descriptor,buf,1023);
        buf[n>0?n:0] = 0;
        printf("%i bytes read from device...\n", n);
        printf("buffer contents: %s\n", buf);
        tcflush(file_descriptor, TCIFLUSH);
        
    }

    return 0;
}
