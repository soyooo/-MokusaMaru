#include "mbed.h"
#include "MD2pin.h"
//MDDのコンストラクタ実行と同時にpinをインスタンスする
MD2pin::MD2pin(PinName p, PinName d, int freq) : Pwm(p),Dire(d)
{
    abs_max_output = 1.0;
    Pwm.period(1.0 / freq);
    Pwm.write(0);
    Dire = 0;
    pwm_mode = SM;
}

void MD2pin::drive(float output)
{
    switch(pwm_mode)
    {
        case  SM: _driveSM(output); break;
        case LAP: _driveLAP(output); break;
        default: break;
    }
}

void MD2pin::_driveSM(float output)
{
    if(_abs(output) > abs_max_output)
    {
        Pwm = 0;
        Dire = 0;
    }    
    else if(output > abs_max_output * 0.1)
    {
        Pwm = output / abs_max_output;
        Dire = 0;
    }
    else if(output < -abs_max_output * 0.1)
    {
        Pwm = -output / abs_max_output;
        Dire = 1;
    }
    else
    {
        Pwm = 0;
        Dire = 0;
    }    
}

void MD2pin::_driveLAP(float output)
{
    if(_abs(output) > abs_max_output)
    {
        Pwm = 0;
        Dire = 0;
    }    
    else if(output > abs_max_output * 0.1)
    {
        Pwm = 0.5 + (output / abs_max_output / 2);
        Dire = 1;
    }
    else if(output < -abs_max_output * 0.1)
    {
        Pwm = 0.5 - (output / abs_max_output / 2);
        Dire = 1;
    }
    else
    {
        Pwm = 0;
        Dire = 0;
    }    
}

float MD2pin::_abs(float val)
{
    if(val >= 0)
        return val;
    else return -val;
}