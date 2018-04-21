#include "mbed.h"
#include "MDD.h"
#include "imu.h"
#include "RotaryEncoder.h"
#include "CSV.h"
#include "SBUS.h"
#include "RobotPosure.h"
#include "PID.h"

//オンボードLEDの使用宣言
DigitalOut LED[] = {DigitalOut(LED1),
                    DigitalOut(LED2),
                    DigitalOut(LED3),
                    DigitalOut(LED4),};
//デバック用シリアル
Serial pc(USBTX, USBRX);
Serial XBEE(p28, p27);

//propo reciever とSBUSで通信する
SBUS propo(p13, p14);

//３輪オムニ逆運動学モデルのヤコビアン
float jacobian[4][3] ={{0.5,  0.866, -1},
                       {0.5, -0.866, -1},
                       { -1,      0, -1},
                       {  0,      0,  0}};

//ロボットの移動に関する計算をするやつ
RobotPosure robot(jacobian, 0.95);

//IMUと通信計算するやつ
IMU imu(0.005, p9, p10);

//MD用の信号ピン設定
  MDD Motor[5] = {  MDD(p22, p21),
                    MDD(p23, p29),
                    MDD(p24, p30),
                    MDD(p25, p19),
                    MDD(p26, p20)
                    };

//エンコーダーピン設定
RotaryEncoder enc[5] = {  RotaryEncoder(p5,p6),
                          RotaryEncoder(p18,p17),
                          RotaryEncoder(p16,p15),
                          RotaryEncoder(p12,p11),
                          RotaryEncoder(p8,p7)
                        };

typedef struct controller
{
    int LX, LY, RX, RY;
    bool r1;
}ctrlr;

int main()
{
    //yow角補正用のPID計算するやつ
    PID pidRobotYow(0.01, 0, 0, 0.01, 0.95);

    //タイマー３の優先度を最低にする
    NVIC_SetPriority(TIMER3_IRQn, 100);

    //IMUのキャリブレーション
    imu.performCalibration();
    imu.startAngleComputing();

    robot.imu_yow = &imu.angle[2];
    pidRobotYow.sensor = &imu.angle[2];
    pidRobotYow.start();

    while(1)
    {
        float robot_velocity[3] = {cmd.LX, cmd.LY, cmd.RX};

        if(robot_velocity[2] == 0)
            robot_velocity[2] = pidRobotYow.output;
        else *pidRobotYow.target= *pidRobotYow.sensor;

        robot.setVelL(robot_velocity);
        robot.computeWheelVel();
        robot.rescaleWheelVel();

        for(int i = 0; i < 3; i++)
            Motor[i].drive(robot.wheel_vel[i]);

        wait(0.002);
    }
}
