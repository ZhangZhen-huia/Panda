#ifndef __WAYPOINT_TASK__H
#define __WAYPOINT_TASK__H

#include "main.h"

typedef struct
{
//	float Way_X;
//	float Way_Y;
	float Plo_Angle;
	float Plo_Length;
}Waypoints_t;


extern Waypoints_t Waypoint[50];
void Waypoint_Write(uint8_t number,float Plo_Angle,float Plo_Length);
void Waypoint_Init(void);

#endif


