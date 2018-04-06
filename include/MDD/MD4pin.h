#ifndef MD4_H
#define MD4_H

#include "mbed.h"

class MD4pin
{
public:
    float abs_max_output;
    MD4pin(PinName pa, PinName pb, int freq = 10000);
    void drive(float output);
private:
    //デフォルトコンストラクタが必ず必要なので、MDDのコンストラクタ実行と同時にインスタンスする
    PwmOut Pwm_a;
    PwmOut Pwm_b;
    float _abs(float val);
};
#endif