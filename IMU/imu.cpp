#include"imu.h"
#include"mbed.h"
#include "mpu9250_i2c.h"
#include "math.h"

IMU::IMU(float t, PinName sla , PinName sld) : i2c(sla, sld), imu(i2c, AD0_LOW)
{
    delta_t = t;
    imu.setAcc();
    imu.setGyro(_1000DPS);
}

void IMU::startAngleComputing()
{
    angleTimer.attach(this, &IMU::_computeAngle, delta_t);
}

void IMU::stopAngleComputing()
{
    angleTimer.detach();
}

void IMU::_updataImuValue()
{
    imu.getAcc(&acc[0], &acc[1], &acc[2]);
    imu.getGyro(&gyro[0], &gyro[1], &gyro[2]);
    angle_acc[0] = 180.0 / PI * atan2(acc[0], sqrt(pow(acc[1], 2) + pow(acc[2], 2) ) );
    angle_acc[1] = 180.0 / PI * atan2(acc[1], sqrt(pow(acc[0], 2) + pow(acc[2], 2) ) );
}

void IMU::_computeAngle()
{
    angle[0] = 0.95 * (pre_angle[0] + gyro[0] * delta_t) + 0.05 * angle_acc[0];
    angle[1] = 0.95 * (pre_angle[1] + gyro[1] * delta_t) + 0.05 * angle_acc[1];
    angle[2] += gyro[2] * delta_t;
    pre_angle[0] = angle[0];
    pre_angle[1] = angle[1];
    _updataImuValue();
}

void IMU::performCalibration()
{
     float raw_senser[3][3] = {0};
     float offset[3][3] = {0};
    wait(0.1);
    for(int i = 0; i < 1000; i++)
    {
        imu.getAcc (&raw_senser[0][0], &raw_senser[0][1], &raw_senser[0][2]);
        imu.getGyro(&raw_senser[1][0], &raw_senser[1][1], &raw_senser[1][2]);
        imu.getMag (&raw_senser[2][0], &raw_senser[2][1], &raw_senser[2][2]);
        for(int k = 0; k < 3; k++)
            for(int p = 0; p < 3; p++)
                offset[k][p] += raw_senser[k][p] * 0.001;
    }

    imu.setOffset(  offset[0][0], offset[0][1], offset[0][2] - 1.0,
                    offset[1][0], offset[1][1], offset[1][2],
                    offset[2][0], offset[2][1], offset[2][2]);
}
