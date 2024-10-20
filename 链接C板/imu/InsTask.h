#ifndef INSTASK_H
#define INSTASK_H

#include "main.h"
#include "def.h"
#include "bmi088.h"

#define dt 0.01f
#define halfdt 0.005f

#define wx  ImuRobot.Gyr[0]
#define wy  ImuRobot.Gyr[1]
#define wz  ImuRobot.Gyr[2]
#define gx  (-ImuRobot.Acc[0])
#define gy  (-ImuRobot.Acc[1])
#define gz  (-ImuRobot.Acc[2])

#define q0 ImuRobot.q[0]
#define q1 ImuRobot.q[1]
#define q2 ImuRobot.q[2]
#define q3 ImuRobot.q[3]

#define q0q0 q0*q0
#define q1q1 q1*q1
#define q2q2 q2*q2
#define q3q3 q3*q3
#define q1q3 q1*q3
#define q2q0 q2*q0
#define q1q0 q1*q0
#define q2q3 q2*q3

#define vx ImuRobot.v[0] 
#define vy ImuRobot.v[1] 
#define vz ImuRobot.v[2]

#define ex ImuRobot.Error[0]
#define ey ImuRobot.Error[1]
#define ez ImuRobot.Error[2]

#define iex ImuRobot.iError[0]
#define iey ImuRobot.iError[1]
#define iez ImuRobot.iError[2]

#define Looptimes 5000  

const ImuData *getImuRobot(void);
extern ImuData ImuRobot;
void InsTask(void const * argument);


#endif
