#ifndef ROBOT_POSURE_H
#define ROBOT_POSURE_H
#include "mbed.h"
#include "PID.h"

class RobotPosure
{
public:
    RobotPosure(float matrix[4][3], float max = 1.0);

    float odometry[2];
    float *imu_yow;      //IMUからの参照

    float robot_vel_local[3];    //ロボット主観 (vx, vy, w)

    float driver_jacobian[4][3];  //ロボット速度→ホイール速度のヤコビアン
    float odometer_jacobian[4][3];   //オドメトリホイール速度→ロボット速度のヤコビアン

    float wheel_vel[4];
    float abs_max_vel;   //ホイールの最大回転速度   単位は知らない

    void setVelL(float vel[3]);
    void setVelG(float vel[3]);
    void computeWheelVel();
    void rescaleWheelVel();

    void setOdometerParameter(float matrix[3][3], float *encoder[3], float r, float l);
    void startComputingOdometry(float delta_t = 0.1, float x0 = 0, float y0 = 0);

private:
    Ticker odometerInterrupt;
    float last_imu_yow;
    float wheel_radius;
    float robot_radius;
    float *rotation_num[3];
    float last_rotation_num[3];
    void _guard(float* val);
    void _computeOdometry();
    float _abs(float val);
};

#endif
