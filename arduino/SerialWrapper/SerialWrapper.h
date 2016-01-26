#ifndef SERIALSEND_H
#define SERIALSEND_H

#include "Arduino.h"


#define DEFAULT_BAUD 9600

class SerialWrapper
{

private:

    int _baud_rate;

    int _ultrasonic_count;
    int _infrared_count;
    int _encoder_count;


    int _buf_size;
    char* _buf;
    
public:

    SerialWrapper();
    SerialWrapper(int baud, int us, int inf, int enc);

    void send();
    void receive();

    void update_buffer();
    void parse_buffer();

};

#endif
