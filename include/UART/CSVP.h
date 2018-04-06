#ifndef CSVP_H
#define CSVP_H
#include "mbed.h"

class CSVP
{
public:
    CSVP(PinName TX, PinName RX, int bps = 9600);
    int rcvd[100];
    
private:
    Serial myserial;
    char buf[512];
    int count;
    void _serialEvent();
};
#endif