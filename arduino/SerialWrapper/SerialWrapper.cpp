#include "SerialWrapper.h"

SerialWrapper::SerialWrapper() 
{
    _baud_rate = DEFAULT_BAUD;
    _ultrasonic_count = 3;
    _infrared_count = 2;
    _encoder_count = 2;


    //calculate buffer size and allocate memory for buffer
    _buf_size = 0;
    _buf = NULL;

    Serial.begin(_baud_rate);

}