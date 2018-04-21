#include "mbed.h"
#include "robotPosure.h"
#include "PID.h"
#define PI 3.141592653589793

/*
* コンストラクタ
* ヤコビアンと最大回転速度を定義する
*/
robotPosure::robotPosure(float matrix[4][3], float max)
{
    absMaxVelocity = max;
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 3; j++)
            driveJacobian[i][j] = matrix[i][j];
}

/*
* フィールド座標系でのロボット速度を設定する
*/
void robotPosure::setVelocityG(float velocity[3])
{
    float COS = cos(*imuYow * 2 * PI / 360);
    float SIN = sin(*imuYow * 2 * PI / 360);
    robotVelocityL[0] =  COS * velocity[0] + SIN * velocity[1];
    robotVelocityL[1] = -SIN * velocity[0] + COS * velocity[1];
    robotVelocityL[2] = velocity[2];
}

/*
* ロボット主観でのロボット速度を設定する
*/
void robotPosure::setVelocityL(float velocity[3])
{
    robotVelocityL[0] =  velocity[0];
    robotVelocityL[1] =  velocity[1];
    robotVelocityL[2] =  velocity[2];
}

/*
* ヤコビアンを使ってロボット速度をホイール速度に変換する
*/
void robotPosure::computeWheelVelocity()
{
    for(int i = 0 ; i < 4; i++)
    {
        wheelVelocity[i] = 0;
        for(int j = 0; j < 3; j++)
           wheelVelocity[i] += driveJacobian[i][j] * robotVelocityL[j];
    }
}
/*
* ホイール速度が最大回転速度を超えた場合、ホイールの速度全体を縮小する
*/
void robotPosure::rescaleWheelVelocity()
{
    float maxVal = _abs(wheelVelocity[0]);

    for(int i = 1; i < 4; i++)
        if(maxVal < _abs(wheelVelocity[i]))
            maxVal = _abs(wheelVelocity[i]);
    if(maxVal > absMaxVelocity)
    {
        for(int i = 0; i < 4 ; i++)
        {
            wheelVelocity[i] *= absMaxVelocity / maxVal;
            guard(&wheelVelocity[i]);
        }
    }
}
/*
* ガードする
*/
void robotPosure::guard(float* val)
{
    if(*val > absMaxVelocity)
        *val = absMaxVelocity;
    else if(*val < -absMaxVelocity)
        *val = -absMaxVelocity;
}

/*
* オドメトリ計算のためのパラメータを設定する
*/
void robotPosure::setOdometerParameter(float matrix[3][3], float *encoder[3], float r, float l)
{
     for(int i = 0; i < 3; i++)
     {
       rotationNum[i] = encoder[i];
       for(int j = 0; j < 3; j++)
            odometerJacobian[i][j] = matrix[i][j];
     }
     wheelRadius  = r;
     robotRadius  = l;
}

/*
* 初期条件を設定して、オドメトリ計算を始める
* 以後、オドメトリは自動更新される
*/
void robotPosure::startComputingOdometry(float deltaT, float x0, float y0)
{
    odometerInterrupt.attach(this, &robotPosure::computeOdometry, deltaT);
    odometry[0] = x0;
    odometry[1] = y0;
}

/*
* オドメトリを計算する
* タイマーで定期的に実行される
*/
void robotPosure::computeOdometry()
{
    float deltaX, deltaY;
    float deltaRotationNum[3];

    for(int i = 0; i < 3; i++)
    {
        deltaRotationNum[i] = *rotationNum[i] - lastRotationNum[i];
        lastRotationNum[i] = *rotationNum[i];
    }

    for(int i = 0 ; i < 2; i++)
    {
        deltaX += 2 * PI * wheelRadius * deltaRotationNum[i] * odometerJacobian[0][i];
        deltaY += 2 * PI * wheelRadius * deltaRotationNum[i] * odometerJacobian[1][i];
    }

    float COS = cos(lastImuYow * 2 * PI / 360);
    float SIN = sin(lastImuYow * 2 * PI / 360);

    odometry[0] += deltaX * COS - deltaY * SIN;
    odometry[1] += deltaX * SIN + deltaY * COS;

    lastImuYow = *imuYow;
}


float robotPosure::_abs(float val)
{
    if(val >= 0)
        return val;
    else return -val;
}
