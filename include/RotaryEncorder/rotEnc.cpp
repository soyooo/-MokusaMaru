#include "mbed.h"
#include "rotEnc.h"

Enc::Enc(PinName a, PinName b, int count_per_rotation) : encA(a), encB(b)
{
    count = 0;
    count_per_interrupt = 1.0 / count_per_rotation;
    encA.rise(this, &Enc::_aRaise);
    encA.fall(this, &Enc::_aFall);
    encB.rise(this, &Enc::_bRaise);
    encB.fall(this, &Enc::_bFall);  
}

void Enc::changeDirection()
{
    count_per_interrupt *= -1;
}

void Enc::defineNowCount(float n)
{
    count = n;
}

void Enc::_aRaise()
{
    if(!encB)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void Enc::_aFall()
{
    if(encB)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void Enc::_bRaise()
{
    if(encA)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}

void Enc::_bFall()
{
    if(!encA)
        count += count_per_interrupt;
    else count -= count_per_interrupt;
}  