#include "mbed.h"
#include "PID.h"

PID::PID(float p, float i, float d, float t, float max)
{
    kp = p; ki = i; kd = d; delta_t = t;
    abs_max_output = 1.0;
    integral = 0;
}

void PID::start()
{
    pidTimer.attach(this, &PID::_compute, delta_t);
}

void PID::stop()
{
    pidTimer.detach();
}

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

void PID::reset()
{
    integral = 0;
    error[0] = 0;
    error[1] = 0;
}

float PID::_gurd(float val, float abs_max)
{
    if(val > abs_max)
        return abs_max;
    else if(val < -abs_max)
        return -abs_max;
    else return val;
}
