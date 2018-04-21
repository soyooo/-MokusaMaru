#ifndef SBUS_H
#define SBUS_H
#include "mbed.h"

#define SBUS_SYNCBYTE 0x0F

class SBUS
{
public:
    SBUS(PinName TX, PinName RX);
    int chData[18];
    
    bool isFailSafe();
    int getStickVal(int axis);
    int getSwitchVal(int parm);
    
private:
    Serial myserial;
    bool failSafe;
    char rcvData[25];
    int16_t getData(uint8_t ch);
    unsigned int dataIndex;
    void _serialEvent();
    float constrain(float in, float min, float max);
    float map(float in, float inMin, float inMax, float outMin, float outMax);
};
#endif