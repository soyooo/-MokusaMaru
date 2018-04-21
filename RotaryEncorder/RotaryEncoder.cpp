#include "mbed.h"
#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(PinName a, PinName b, int count_per_rotation) : pinA(a), pinB(b)
{
    count = 0;
    count_per_interrupt = 1.0 / count_per_rotation;
    pinA.rise(this, &RotaryEncoder::_aRaise);
    pinA.fall(this, &RotaryEncoder::_aFall);
    pinB.rise(this, &RotaryEncoder::_bRaise);
    pinB.fall(this, &RotaryEncoder::_bFall);
}

void RotaryEncoder::changeDirection()
{
    count_per_interrupt *= -1;
}

void RotaryEncoder::defineNowCount(float n)
{
    count = n;
}

void RotaryEncoder::_aRaise()
{
    if(!pinB)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void RotaryEncoder::_aFall()
{
    if(pinB)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void RotaryEncoder::_bRaise()
{
    if(pinA)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void RotaryEncoder::_bFall()
{
    if(!pinA)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}
