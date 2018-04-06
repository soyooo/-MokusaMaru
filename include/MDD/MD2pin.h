#ifndef MD2_H
#define MD2_H

#include "mbed.h"

#define SM  0
#define LAP 1

class MD2pin
{
public:
    float abs_max_output;
    MD2pin(PinName p, PinName d, int freq = 10000);
    void drive(float output);
    int pwm_mode;
private:
    //デフォルトコンストラクタが必ず必要なので、MDDのコンストラクタ実行と同時にインスタンスする
    PwmOut Pwm;
    DigitalOut Dire;
    void _driveSM(float output);
    void _driveLAP(float output);
    float _abs(float val);
};
#endif

