#ifndef _IMU_H_
#define _IMU_H_

#include "mbed.h"
#include "mpu9250_i2c.h"

#define PI 3.141592653589793f

class IMU
{
public:
    IMU(float t, PinName sla = p9, PinName sld = p10);

    void performCalibration();
    void startAngleComputing();
    void stopAngleComputing();

    float angle[3]; //角度[roll,pitch,yow]　[deg]
    float pre_angle[3]; //前回角度[roll,pitch,yow]　[deg]
    float angle_acc[3]; //加速度センサーのみで計算された角度[roll,pitch,yow]　[deg]
    float acc[3]; //加速度[x,y,z] [m/^2]
    float gyro[3];//角速度[roll,pitch,yow] [deg/sec]
    float mag[3]; //磁束密度?[x,y,z]  [??]

private:
    I2C i2c;
    mpu9250 imu;
    Ticker angleTimer;
    float delta_t;
    void _updataImuValue();
    void _computeAngle();
};
#endif
