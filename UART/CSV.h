#ifndef CSV_H
#define CSV_H
#include "mbed.h"

class CSV
{
public:
    CSV(PinName TX, PinName RX, int bps = 9600);
    int valueData[100];
    int getValue(int num);
private:
    Serial myserial;
    char rcvData[512];
    unsigned int dataIndex;
    void _serialEvent();
};
#endif
