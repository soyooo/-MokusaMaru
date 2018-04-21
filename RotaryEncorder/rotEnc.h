#ifndef ROTENC_H
#define ROTENC_H

#include "mbed.h"

class Enc
{
public:
    float count;
    Enc(PinName a, PinName b, int count_per_rotation = 400);
    void changeDirection();
    void defineNowCount(float n);
private:
    InterruptIn encA;
    InterruptIn encB;
    float count_per_interrupt;
    void _aRaise();
    void _aFall();
    void _bRaise();
    void _bFall();
};

#endif