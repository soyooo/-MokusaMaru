#ifndef ROBOT_POSURE_H
#define ROBOT_POSURE_H
#include "mbed.h"
#include "PID.h"


class robotPosure
{
public:
    robotPosure(float matrix[4][3], float max = 1.0);

    float odometry[2];
    float *imuYow;      //IMUからの参照
    float targetPosure[3];

    float robotVelocityL[3];    //ロボット主観 (vx, vy, w)

    float driveJacobian[4][3];  //ロボット速度→ホイール速度のヤコビアン
    float odometerJacobian[4][3];   //オドメトリホイール速度→ロボット速度のヤコビアン

    float wheelVelocity[4];
    float absMaxVelocity;   //ホイールの最大回転速度   単位は知らない

    void setVelocityL(float velocity[3]);
    void setVelocityG(float velocity[3]);
    void computeWheelVelocity();
    void rescaleWheelVelocity();

    void setOdometerParameter(float matrix[3][3], float *encoder[3], float r, float l);
    void startComputingOdometry(float deltaT = 0.1, float x0 = 0, float y0 = 0);

private:
    Ticker odometerInterrupt;
    void guard(float* val);
    void computeOdometry();
    float _abs(float val);

    float lastImuYow;
    float wheelRadius;
    float robotRadius;
    float *rotationNum[3];
    float lastRotationNum[3];
    float a;
};

#endif
