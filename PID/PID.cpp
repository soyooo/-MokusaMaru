#include "mbed.h"
#include "PID.h"

/*p,i,d:ゲイン設定　t:制御ループ時間 max:ourputの最大値*/
PID::PID(float p, float i, float d, float t, float max)
{
    kp = p; ki = i; kd = d; delta_t = t;
    abs_max_output = 1.0;
    integral = 0;
}

/*計算のための割り込み開始*/
void PID::start()
{
    pidTimer.attach(this, &PID::_compute, delta_t);
}

/*計算のための割り込み停止*/
void PID::stop()
{
    pidTimer.detach();
}

/*現在保持している計算データをリセット*/
void PID::reset()
{
    integral = 0;
    error[0] = 0;
    error[1] = 0;
}

/*計算する　タイマーで回される*/
void PID::_compute()
{
    error[0] = *target - *sensor;

    float proportion, differential, myoutput;

    proportion    = kp * error[0];
    integral     += ki * error[0] * delta_t;
    differential  = kd * (error[0] - error[1]) / delta_t;

    error[1] = error[0];

    integral = _gurd(integral, abs_max_output);
    myoutput = proportion + integral + differential;
    myoutput = _gurd(myoutput, abs_max_output);
    output = myoutput;
}


float PID::_gurd(float val, float abs_max)
{
    if(val > abs_max)
        return abs_max;
    else if(val < -abs_max)
        return -abs_max;
    else return val;
}
