#ifndef DEF_H_
#define DEF_H_

#include "main.h"
#include "arm_math.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"

typedef struct
{
	
	float GyrOffset[3];
	float Error[3];
	float iError[3];
	
	float GyrOrign[3];
	
	float Gyr[3];
	float Acc[3];
	float Temperate;
	
	float Yaw;
	float Pitch;
	float Roll;
	
	float TotalYaw;
	float DYaw;
	float LastYaw;
	float YawTo360;
	
	float q[4];
	float v[3];
	float Vn[3]; // 惯性系加速度
	
	float ImuKp;
	float ImuKi;
	
	float InitFlag;
	
}ImuData;




#endif
