#include "mbed.h"
#include "MDD.h"
#include "imu.h"
#include "rotaryEncoder.h"
#include "CSVP.h"
#include "SBUS.h"
#include "robotPosure.h"
#include "PID.h"

//オンボードLEDの使用宣言
DigitalOut LED[] = {DigitalOut(LED1),
                    DigitalOut(LED2),
                    DigitalOut(LED3),
                    DigitalOut(LED4),};
//デバック用シリアル
Serial pc(USBTX, USBRX);
Serial XBEE(p28, p27);

//csv形式で通信する
CSVP xbee(p28, p27);

//propo reciever とSBUSで通信する
SBUS propo(p13, p14);

//３輪オムニ逆運動学モデルのヤコビアン
float jacobian[4][3] ={{0.5,  0.866, -1},
                       {0.5, -0.866, -1},
                       { -1,      0, -1},
                       {  0,      0,  0}};

//ロボットの移動に関する計算をするやつ
robotPosure robot(jacobian, 0.95);

//IMUと通信計算するやつ
IMU imu(0.005, p9, p10);

//MD用の信号ピン設定
MDD Motor[5] = { MDD(p22, p21),
                    MDD(p23, p29),
                    MDD(p24, p30),
                    MDD(p25, p19),
                    MDD(p26, p20)
                    };

//エンコーダーピン設定
rotaryEncoder enc[5] = {  rotaryEncoder(p5,p6),
                rotaryEncoder(p18,p17),
                rotaryEncoder(p16,p15),
                rotaryEncoder(p12,p11),
                rotaryEncoder(p8,p7)
            };

typedef struct controller
{
    int LX, LY, RX, RY;
    bool r1;
}ctrlr;

int main()
{
    //yow角補正用のPID計算するやつ
    PID pidYow(0.01, 0, 0, 0.01, 0.95);

    //タイマー３の優先度を最低にする
    NVIC_SetPriority(TIMER3_IRQn, 100);

    //IMUのキャリブレーション
    imu.doOffset();
    imu.startComputingAngle();

    robot.imuYow = &imu.angle[2];
    pidYow.sensor = &imu.angle[2];
    pidYow.target = &robot.targetPosure[2];
    pidYow.start();

    while(1)
    {
        ctrlr cmd;
        cmd.LX = propo.getStickVal(0);
        cmd.LY = propo.getStickVal(1);
        cmd.RX = propo.getStickVal(2);
        cmd.RY = propo.getStickVal(3);

        float velocity[3] = {cmd.LX, cmd.LY, cmd.RX};

        if(velocity[2] == 0)
            velocity[2] = pidYow.output;
        else *pidYow.target= *pidYow.sensor;

        robot.setVelocityL(velocity);
        robot.computeWheelVelocity();
        robot.rescaleWheelVelocity();

        pc.printf("%.3f  ", imu.angle[2]);

        for(int i = 0; i < 3; i++)
            Motor[i].drive(robot.wheelVelocity[i]);

        pc.printf("\n");
        wait(0.02);
    }
}
