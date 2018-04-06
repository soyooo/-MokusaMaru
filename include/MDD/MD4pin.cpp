#include "mbed.h"
#include "MD4pin.h"
//MDDのコンストラクタ実行と同時にpinをインスタンスする
MD4pin::MD4pin(PinName pa, PinName pb, int freq): Pwm_a(pa),Pwm_b(pb)
{
    abs_max_output = 1.0;
    Pwm_a.period(1.0 / freq);
    Pwm_a.write(0);
    Pwm_b.period(1.0 / freq);
    Pwm_b.write(0);
}
void MD4pin::drive(float output)
{
    if(_abs(output) > abs_max_output)
    {
        Pwm_a.write(0);
        Pwm_b.write(0);
    }    
    else if(output > abs_max_output * 0.1)
    {
        Pwm_a.write(output / abs_max_output);
        Pwm_b.write(0);
    }
    else if(output < -abs_max_output * 0.1)
    {
        Pwm_a.write(0);
        Pwm_b.write(-output / abs_max_output);
    }
    else
    {
        Pwm_a.write(0);
        Pwm_b.write(0);
    }    
}
float MD4pin::_abs(float val)
{
    if(val >= 0)
        return val;
    else return -val;
}