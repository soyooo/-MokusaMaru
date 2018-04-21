#ifndef PID_H
#define PID_H

#include "mbed.h"

//PID計算機
/**/
class PID
{
public:
    PID(float p, float i, float d, float t, float max = 1.0);
    void start();
    void stop();
    void reset();

    float output; //計算された出力値が入る　

    float abs_max_output; //outputの100%を定義
    float *sensor, *target; //センサーと目標値を示す変数のポインタ 外側から引っ張ってくる

private:
    Ticker pidTimer;
    float kp, ki, kd, delta_t;
    float integral;
    float error[2];
    void _compute();
    float _gurd(float val, float abs_max);
};
#endif
