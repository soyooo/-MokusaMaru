//モクサマルmainNode
#include "mbed.h"
#include "MD2pin.h"
#include "imu.h"
#include "rotEnc.h"
#include "CSVP.h"
#include "robotPosure.h"
#include "PID.h"

DigitalOut LED[] = {DigitalOut(LED1),
                    DigitalOut(LED2),
                    DigitalOut(LED3),
                    DigitalOut(LED4),};

Serial pc(USBTX, USBRX);
Serial subNode(p13, p14);
Serial XBEE(p28, p27);
CSVP xbee(p28, p27);

float matrix[4][3] ={{0.5,  0.866, -1},
                     {0.5, -0.866, -1},
                     { -1,      0, -1},
                     {  0,      0,  0}};

robotPosure robot(matrix, 0.95);

IMU imu(0.005, p9, p10);

MD2pin Motor[5] = { MD2pin(p22, p21),
                    MD2pin(p23, p29),
                    MD2pin(p24, p30),
                    MD2pin(p25, p19),
                    MD2pin(p26, p20)
                    };

Enc enc[5] = {  Enc(p5,p6),
                Enc(p18,p17),
                Enc(p16,p15),
                Enc(p12,p11),
                Enc(p8,p7)
            };

typedef struct controller
{
    int LX, LY, RX, RY;
    bool r1;
}ctrlr;


PID pidYow(0.01, 0, 0, 0.01, 0.95);
ctrlr getControllerData();

int main()
{
    NVIC_SetPriority(TIMER3_IRQn, 100);
    imu.doOffset();
    imu.startComputingAngle();

    float odometerJacobian[3][3] = {{0.33333,  0.33333, -0.66667},
                                    {0.57735, -0.57735, 0       },
                                    {-0.33333, -0.33333, -0.33333}};

    float *encoder[3] = {&enc[0].count, &enc[1].count, &enc[2].count};
    robot.imuYow = &imu.angle[2];
    robot.setOdometerParameter(odometerJacobian, encoder, 5, 19.3);
    robot.startComputingOdometry();

    pidYow.sensor = &imu.angle[2];
    pidYow.target = &robot.targetPosure[2];
    pidYow.start();

    while(1)
    {
        /*
        ctrlr ctrlr;
        ctrlr.LX = xbee.rcvd[0] * 0.01;
        ctrlr.LY = xbee.rcvd[1] * 0.01;
        ctrlr.RX = xbee.rcvd[2] * 0.01;
        ctrlr.RY = xbee.rcvd[3] * 0.01;
        ctrlr.r1 = bool(xbee.rcvd[4]);
        */

        //LED[0] = ctrlr.r1;

        //float velocity[3] = {ctrlr.LX, ctrlr.LY, ctrlr.RX};
        float velocity[3] = {0, 0, 0};

        if(velocity[2] == 0)
            velocity[2] = pidYow.output;
        else *pidYow.target= *pidYow.sensor;

        robot.setVelocityL(velocity);
        robot.computeWheelVelocity();
        robot.rescaleWheelVelocity();

        pc.printf("%.3f  ", imu.angle[2]);
        pc.printf("%.3f  ", robot.odometry[0]);
        pc.printf("%.3f  ", robot.odometry[1]);
        pc.printf("%.3f  ", velocity[2]);

        for(int i = 0; i < 3; i++)
        {

            pc.printf("%.3f  ", robot.wheelVelocity[i]);
            Motor[i].drive(robot.wheelVelocity[i]);
        }

        pc.printf("\n");
        wait(0.02);
    }
}

ctrlr getControllerData()
{
    ctrlr ctrlr;
    ctrlr.LX = xbee.rcvd[0] * 0.01;
    ctrlr.LY = xbee.rcvd[1] * 0.01;
    ctrlr.RX = xbee.rcvd[2] * 0.01;
    ctrlr.RY = xbee.rcvd[3] * 0.01;
    ctrlr.r1 = bool(xbee.rcvd[4]);
    return ctrlr;
}
