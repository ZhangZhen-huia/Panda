#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_bmi088.h"
#include "def.h"
#include "instask.h"
#include "tim.h"


ImuData ImuRobot;


//计算浮点数平方根的倒数
//使用了一种称为快速逆平方根（Fast Inverse Square Root, FIS）的算法
//优点是速度快，但是精度不如1.0f / sqrt(num)
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}


float ImuOffset[3] = {-0.00163245865, -0.000815640669, 0.00117061427};

void ImuZero(void);
void ImuInit(void);
float ImuError(void);
void ImuQuaternion(void);

void InsTask(void const * argument)
{	
	ImuZero();
	
	ImuInit();

	for(;;)
	{
		BMI088_read(ImuRobot.GyrOrign, ImuRobot.Acc, &ImuRobot.Temperate);
		
		ImuQuaternion();
		vTaskDelay(10);
	}
}

void ImuZero(void)
{
	float temperate;
	
	for(uint32_t i = 0; i < Looptimes; ++i)
	{	
		BMI088_read(ImuRobot.GyrOrign, ImuRobot.Acc, &temperate);
		ImuRobot.GyrOffset[0] += ImuRobot.GyrOrign[0];
		ImuRobot.GyrOffset[1] += ImuRobot.GyrOrign[1];
		ImuRobot.GyrOffset[2] += ImuRobot.GyrOrign[2];
		HAL_Delay(1);
	}
//	ImuRobot.GyrOffset[0] = ImuOffset[0];
//	ImuRobot.GyrOffset[1] = ImuOffset[1];
//	ImuRobot.GyrOffset[2] = ImuOffset[2];
	ImuRobot.GyrOffset[0] /= Looptimes;
	ImuRobot.GyrOffset[1] /= Looptimes;
	ImuRobot.GyrOffset[2] /= Looptimes;
	
	ImuRobot.q[0] = 1.0f;
	ImuRobot.q[1] = 0.0f;
	ImuRobot.q[2] = 0.0f;
	ImuRobot.q[3] = 0.0f;
	
	
}

void ImuInit(void)
{
	//ImuRobot.CrtlRobot = getCrtlRobot();
	
//	while(1)
//	{
//		if(ImuRobot.CrtlRobot->ChassisInit == Yes)
//			break;
//	}
//	
	ImuZero();
	
	ImuRobot.ImuKp = 0.1f;
	
	for(;;)
	{
		BMI088_read(ImuRobot.GyrOrign, ImuRobot.Acc, &ImuRobot.Temperate);
		ImuQuaternion();
		if(ImuError() < 0.1f)
		{
			ImuRobot.ImuKp = 0.01f;
			break;
		}
		vTaskDelay(10);
	}
	
	ImuRobot.InitFlag = 1;
}

float ImuError(void)
{
	return (ImuRobot.Error[0] * ImuRobot.Error[0] + ImuRobot.Error[1] * ImuRobot.Error[1] + ImuRobot.Error[2] * ImuRobot.Error[2]);
}

void ImuQuaternion(void)
{
//	//0-roll，1-pitch，2-yaw
//	ImuRobot.Gyr[0] = ImuRobot.GyrOrign[0] - ImuRobot.GyrOffset[0];
//	ImuRobot.Gyr[1] = ImuRobot.GyrOrign[1] - ImuRobot.GyrOffset[1];
//	ImuRobot.Gyr[2] = ImuRobot.GyrOrign[2] - ImuRobot.GyrOffset[2];

	//1-roll，0-pitch，2-yaw
	ImuRobot.Gyr[0] = ImuRobot.GyrOrign[0] - ImuRobot.GyrOffset[0];
	ImuRobot.Gyr[1] = ImuRobot.GyrOrign[1] - ImuRobot.GyrOffset[1];
	ImuRobot.Gyr[2] = ImuRobot.GyrOrign[2] - ImuRobot.GyrOffset[2];
	vx = -2.0f * (q1q3 + q2q0);
	vy = 2.0f * (q1q0 - q2q3);
	vz = -q0q0 + q1q1 + q2q2 - q3q3;
	
	ImuRobot.Vn[0] = 2 * (-gz * q0 * q2 + gy * q1 * q2 + gy * q0 * q3 + gz * q1 * q3) + gx * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);//惯性系下的加速度
	ImuRobot.Vn[1] = 2 * (gz * q0 * q1 + gx * q1 * q2 - gx * q0 * q3 + gz * q2 * q3) + gy * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
	ImuRobot.Vn[2] = 2 * (-gy * q0 * q1 + gx * q0 * q2 + gx * q1 * q3 + gy * q2 * q3) + gz * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) + 9.76f;	
	
	ex = -gz * vy + gy * vz;
	ey =  gz * vx - gx * vz;
	ez = -gy * vx + gx * vy;
	
	iex += ex;//积累误差
    iey += ey;
    iez += ez;
	
	wx = wx + ex * ImuRobot.ImuKp + iex * ImuRobot.ImuKi; //角速度
    wy = wy + ey * ImuRobot.ImuKp + iey * ImuRobot.ImuKi;
    wz = wz + ez * ImuRobot.ImuKp + iez * ImuRobot.ImuKi;
	
	q0 = q0 - (-q1 * wx - q2 * wy - q3 * wz) * halfdt; //四元数微分方程
    q1 = q1 - (q0 * wx + q3 * wy - q2 * wz) * halfdt;
    q2 = q2 - (-q3 * wx + q0 * wy + q1 * wz) * halfdt;
    q3 = q3 - (q2 * wx - q1 * wy + q0 * wz) * halfdt;

    float QQ = invSqrt(q0q0 + q1q1 + q2q2 + q3q3); // 单位化
    q0 = q0 * QQ;
    q1 = q1 * QQ;
    q2 = q2 * QQ;
    q3 = q3 * QQ;
	
	float yaw 	= atan2(-2.0f * (q1 * q2 - q3 * q0), 1.0f - 2.0f * (q2q2 + q3q3)) * 57.29578f;
    float pitch = asin(2.0f * (q1q3 + q2q0)) * 57.29578f;
    float roll  = atan2(-2.0f * (q2q3 - q1q0), 1.0f - 2.0f * (q1q1 + q2q2)) * 57.29578f;
	
//	ImuRobot.Yaw = yaw;
//	ImuRobot.Pitch = -pitch;
//	ImuRobot.Roll = roll;

	ImuRobot.Yaw = yaw;
	ImuRobot.Pitch = -roll;
	ImuRobot.Roll = pitch;

	ImuRobot.TotalYaw = ImuRobot.Yaw;
	
}

const ImuData *getImuRobot(void)
{
	return &ImuRobot;
}
