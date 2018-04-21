#ifndef PID_H
#define PID_H

#include "mbed.h"

class PID
{
public:
    PID(float p, float i, float d, float t, float max = 1.0);
    void start();
    void stop();
    void reset();
    float output;
    float abs_max_output;
    float *sensor, *target;
    
private:
    Ticker pidTimer;
    float kp, ki, kd, delta_t;
    float integral;
    float error[2];
    void _compute();
    float _gurd(float val, float abs_max);
};
#endif