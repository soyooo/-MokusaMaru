#include "mbed.h"
#include "SBUS.h"

SBUS::SBUS(PinName TX, PinName RX) : myserial(TX, RX)
{
    myserial.baud(100000);
    myserial.format(8, Serial::Even, 2);

    myserial.attach(this, &SBUS::_serialEvent, Serial::RxIrq);
    dataIndex = 0;
    failSafe = true;
}

void SBUS::_serialEvent()
{
    int val = myserial.getc();

    if(dataIndex == 0 && val != SBUS_SYNCBYTE)
        return;

    rcvData[dataIndex] = val;
    dataIndex++;

    if (dataIndex == 25)
    {
        dataIndex = 0;
        if (rcvData[24] != 0x0)
        {
            chData[0] = ((rcvData[1] | rcvData[2] << 8) & 0x07FF);
            chData[1] = ((rcvData[2] >> 3 | rcvData[3] << 5) & 0x07FF);
            chData[2] = ((rcvData[3] >> 6 | rcvData[4] << 2 | rcvData[5] << 10) & 0x07FF);
            chData[3] = ((rcvData[5] >> 1 | rcvData[6] << 7) & 0x07FF);
            chData[4] = ((rcvData[6] >> 4 | rcvData[7] << 4) & 0x07FF);
            chData[5] = ((rcvData[7] >> 7 | rcvData[8] << 1 | rcvData[9] << 9) & 0x07FF);
            chData[6] = ((rcvData[9] >> 2 | rcvData[10] << 6) & 0x07FF);
            chData[7] = ((rcvData[10] >> 5 | rcvData[11] << 3) & 0x07FF);
            chData[8] = ((rcvData[12] | rcvData[13] << 8) & 0x07FF);
            chData[9] = ((rcvData[13] >> 3 | rcvData[14] << 5) & 0x07FF);
            chData[10] = ((rcvData[14] >> 6 | rcvData[15] << 2 | rcvData[16] << 10) & 0x07FF);
            chData[11] = ((rcvData[16] >> 1 | rcvData[17] << 7) & 0x07FF);
            chData[12] = ((rcvData[17] >> 4 | rcvData[18] << 4) & 0x07FF);
            chData[13] = ((rcvData[18] >> 7 | rcvData[19] << 1 | rcvData[20] << 9) & 0x07FF);
            chData[14] = ((rcvData[20] >> 2 | rcvData[21] << 6) & 0x07FF);
            chData[15] = ((rcvData[21] >> 5 | rcvData[22] << 3) & 0x07FF);
            chData[16] = rcvData[23];

            if ((rcvData[23] >> 3) & 0x0001)
                failSafe = true;
            else
                failSafe = false;
        }
    }
}

bool SBUS::isFailSafe()
{
    return failSafe;
}

int16_t SBUS::_getData(uint8_t ch)
{
    return chData[ch];
}

int SBUS::getStickVal(int axis)
{
    switch (axis)
    {
    case 0:
        return _map(_constrain(_getData(3), 368, 1680), 368, 1680, -255, 255);
    case 1:
        return _map(_constrain(_getData(1), 368, 1680), 368, 1680, 255, -255);
    case 2:
        return _map(_constrain(_getData(0), 368, 1680), 368, 1680, -255, 255);
    case 3:
        return _map(_constrain(_getData(2), 368, 1680), 368, 1680, -255, 255);
    default:
        return 0;
    }
}

int SBUS::getSwitchVal(int parm)
{
    if (parm == 0)
        return _map(_constrain(_getData(4), 144, 1904), 144, 1904, 0, 2);
    else if (parm <= 4)
        return _map(_constrain(_getData(parm + 5), 144, 1904), 144, 1904, 0, 2);
    else
        return 0;
}


float SBUS::_constrain(float in, float min, float max)
{
    if(in > max)
        return max;
    else if(in < min)
        return min;
    else return in;
}
float SBUS::_map(float in, float inMin, float inMax, float outMin, float outMax) {
  // check it's within the range
  if (inMin<inMax) {
    if (in <= inMin)
      return outMin;
    if (in >= inMax)
      return outMax;
  } else {  // cope with input range being backwards.
    if (in >= inMin)
      return outMin;
    if (in <= inMax)
      return outMax;
  }
  // calculate how far into the range we are
  float scale = (in-inMin)/(inMax-inMin);
  // calculate the output.
  return outMin + scale*(outMax-outMin);
}
