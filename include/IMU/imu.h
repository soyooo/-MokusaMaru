#ifndef _IMU_H_
#define _IMU_H_

#include "mbed.h"
#include "mpu9250_i2c.h"

#define PI 3.141592653589793f

class IMU
{
public:
    IMU(float t, PinName sla = p9, PinName sld = p10);
    void updataValue();
    void doOffset();
    void startComputingAngle();    
    void stopComputingAngle();
        
    float angle[3];
    float preAngle[3];
    float angleAcc[3];       
    float acc[3];
    float gyro[3];
    float mag[3];
    
private:
    void _computeAngle();
    I2C i2c;
    mpu9250 imu;
    Ticker angleTimer;
    float delta_t;
};
#endif