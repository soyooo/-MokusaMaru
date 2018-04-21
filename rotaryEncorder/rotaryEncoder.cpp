#include "mbed.h"
#include "rotaryEncoder.h"

rotaryEncoder::rotaryEncoder(PinName a, PinName b, int count_per_rotation) : encA(a), encB(b)
{
    count = 0;
    count_per_interrupt = 1.0 / count_per_rotation;
    encA.rise(this, &rotaryEncoder::_aRaise);
    encA.fall(this, &rotaryEncoder::_aFall);
    encB.rise(this, &rotaryEncoder::_bRaise);
    encB.fall(this, &rotaryEncoder::_bFall);
}

void rotaryEncoder::changeDirection()
{
    count_per_interrupt *= -1;
}

void rotaryEncoder::defineNowCount(float n)
{
    count = n;
}

void rotaryEncoder::_aRaise()
{
    if(!encB)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void rotaryEncoder::_aFall()
{
    if(encB)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void rotaryEncoder::_bRaise()
{
    if(encA)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void rotaryEncoder::_bFall()
{
    if(!encA)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}
