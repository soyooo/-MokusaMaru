#include "mbed.h"
#include "RobotPosure.h"
#include "PID.h"
#define PI 3.141592653589793

/*
* コンストラクタ
* ヤコビアンと最大回転速度を定義する
*/
RobotPosure::RobotPosure(float matrix[4][3], float max)
{
    abs_max_vel = max;
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 3; j++)
            driver_jacobian[i][j] = matrix[i][j];
}

/*
* フィールド座標系でのロボット速度を設定する
*/
void RobotPosure::setVelG(float vel[3])
{
    float COS = cos(*imu_yow * 2 * PI / 360);
    float SIN = sin(*imu_yow * 2 * PI / 360);
    robot_vel_local[0] =  COS * vel[0] + SIN * vel[1];
    robot_vel_local[1] = -SIN * vel[0] + COS * vel[1];
    robot_vel_local[2] = vel[2];
}

/*
* ロボット主観でのロボット速度を設定する
*/
void RobotPosure::setVelL(float vel[3])
{
    robot_vel_local[0] =  vel[0];
    robot_vel_local[1] =  vel[1];
    robot_vel_local[2] =  vel[2];
}

/*
* ヤコビアンを使ってロボット速度をホイール速度に変換する
*/
void RobotPosure::computeWheelVel()
{
    for(int i = 0 ; i < 4; i++)
    {
        wheel_vel[i] = 0;
        for(int j = 0; j < 3; j++)
           wheel_vel[i] += driver_jacobian[i][j] * robot_vel_local[j];
    }
}
/*
* ホイール速度が最大回転速度を超えた場合、ホイールの速度全体を縮小する
*/
void RobotPosure::rescaleWheelVel()
{
    float max_vel = _abs(wheel_vel[0]);

    for(int i = 1; i < 4; i++)
        if(max_vel < _abs(wheel_vel[i]))
            max_vel = _abs(wheel_vel[i]);
    if(max_vel > abs_max_vel)
    {
        for(int i = 0; i < 4 ; i++)
        {
            wheel_vel[i] *= abs_max_vel / max_vel;
            _guard(&wheel_vel[i]);
        }
    }
}
/*
* ガードする
*/
void RobotPosure::_guard(float* val)
{
    if(*val > abs_max_vel)
        *val = abs_max_vel;
    else if(*val < -abs_max_vel)
        *val = -abs_max_vel;
}

/*
* オドメトリ計算のためのパラメータを設定する
*/
void RobotPosure::setOdometerParameter(float matrix[3][3], float *encoder[3], float r, float l)
{
     for(int i = 0; i < 3; i++)
     {
       rotation_num[i] = encoder[i];
       for(int j = 0; j < 3; j++)
            odometer_jacobian[i][j] = matrix[i][j];
     }
     wheel_radius  = r;
     robot_radius  = l;
}

/*
* 初期条件を設定して、オドメトリ計算を始める
* 以後、オドメトリは自動更新される
*/
void RobotPosure::startComputingOdometry(float deltaT, float x0, float y0)
{
    odometerInterrupt.attach(this, &RobotPosure::_computeOdometry, deltaT);
    odometry[0] = x0;
    odometry[1] = y0;
}

/*
* オドメトリを計算する
* タイマーで定期的に実行される
*/
void RobotPosure::_computeOdometry()
{
    float delta_x, delta_y;
    float delta_rotation_num[3];

    for(int i = 0; i < 3; i++)
    {
        delta_rotation_num[i] = *rotation_num[i] - last_rotation_num[i];
        last_rotation_num[i] = *rotation_num[i];
    }

    for(int i = 0 ; i < 2; i++)
    {
        delta_x += 2 * PI * wheel_radius * delta_rotation_num[i] * odometer_jacobian[0][i];
        delta_y += 2 * PI * wheel_radius * delta_rotation_num[i] * odometer_jacobian[1][i];
    }

    float COS = cos(last_imu_yow * 2 * PI / 360);
    float SIN = sin(last_imu_yow * 2 * PI / 360);

    odometry[0] += delta_x * COS - delta_y * SIN;
    odometry[1] += delta_x * SIN + delta_y * COS;

    last_imu_yow = *imu_yow;
}


float RobotPosure::_abs(float val)
{
    if(val >= 0)
        return val;
    else return -val;
}
