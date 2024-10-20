#include "chassis_task.h"
#include "pid.h"
int num=10;
int8_t slow_down;
extern int16_t final_yaw;
uint8_t stop_flag;
int8_t over;
int yaw_calibrate_flag;
void Angle_Pid_Trace(chassis_move_t *chassis_move_control_loop,float angle);
void yaw_calibrate_first(chassis_move_t *chassis_move_data);

uint8_t num_1_flag=0;
uint8_t num_3_flag=0;
uint8_t num_7_flag=0;
uint8_t num_9_flag=0;
#include "bsp_imu.h"
/*'''

　　┏┓　　　┏┓+ +
　┏┛┻━━━┛┻┓ + +
　┃　　　　　　　┃ 　
　┃　　　━　　　┃ ++ + + +
 ████━████ ┃+
　┃　　　　　　　┃ +
　┃　　　┻　　　┃
　┃　　　　　　　┃ + +
　┗━┓　　　┏━┛
　　　┃　　　┃　　　　　　　　　　　
　　　┃　　　┃ + + + +
　　　┃　　　┃
　　　┃　　　┃ +  神兽保佑
　　　┃　　　┃    代码无bug　　
　　　┃　　　┃　　+　　　　　　　　　
　　　┃　 　　┗━━━┓ + +
　　　┃ 　　　　　　　┣┓
　　　┃ 　　　　　　　┏┛
　　　┗┓┓┏━┳┓┏┛ + + + +
　　　　┃┫┫　┃┫┫
　　　　┗┻┛　┗┻┛+ + + +
'''*/

/*---------------------------------------------------------------------------------------------------------*/
/*																				|	控制模式	|																											 */
/*					遥控器左边	上：遥控器模式											右边：上：原地小陀螺																 */
/*										中：底盘无力模式													中：全向移动																	 */
/*										下：自动模式															下：急停 or 无力															 */
/*---------------------------------------------------------------------------------------------------------*/

chassis_move_t chassis_move;//底盘运动数据

float x[2]={120,-120};
float y[2]={120,-120};

/*--------------------------------------------------遥控器模式-----------------------------------------------*/
void Pid_caculate(chassis_move_t *chassis_move_control_loop);

// 遥控器模式所有初始化
static void remote_control_chassis_init(chassis_move_t* chassis_move_init);

//遥控器控制模式，获取8个电机的速度并且进行PID计算
static void chassis_remote_control_loop(chassis_move_t *chassis_move_control_loop);

//舵轮6020角度计算,运用atn2函数得到  PI 到 PI 的角度
void Remote_chassis_AGV_wheel_angle(chassis_move_t *chassis_move_control_loop);


//等待6020角度转到位在转3508
void angle_judge(chassis_move_t *chassis_move_control_loop);

//底盘和遥控器模式选择函数
void Chassis_mode_change(chassis_move_t *chassis_move_rc_to_mode);

//遥控器数值获取加死区限制
void Chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_mode);


//pid计算
void Remote_pid_caculate(chassis_move_t *chassis_move_control_loop);
//机器人坐标
void Robot_coordinate(chassis_speed_t * wrold_speed, fp32 angle);//确定底盘运动速度，世界坐标
static void Chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop);

/*-----------------------------------------------------自动------------------------------------------------------*/
// 自动模式所有初始化
static void automatic_control_chassis_init(chassis_move_t* chassis_move_init);

//自动模式下舵轮6020解算
static void Auto_chassis_AGV_wheel_angle(chassis_move_t *chassis_move_control_loop);

//自动模式下舵轮3508解算
static void Auto_chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop);

//PID计算
static void Auto_pid_caculate(chassis_move_t *chassis_move_control_loop);

//计算当前位置Y坐标与目标位置的Y坐标的差值
float Chassis_Me_To_Target_Y_Distance(chassis_move_t *chassis_move_data);

//计算当前位置X坐标与目标位置的X坐标的差值
float Chassis_Me_To_Target_X_Distance(chassis_move_t *chassis_move_data);

//计算当前位置与目标位置的差值
float Chassis_Me_To_Target_Distance(chassis_move_t *chassis_move_data);

//计算当前位置角度与目标位置的角度的差值
int16_t Chassis_Me_To_Target_Angle(chassis_move_t *chassis_move_data);

//极坐标转直角坐标
void Polar_To_Stright(chassis_move_t *chassis_move_data,float plo_angle,float plo_length);

void limit_speed(chassis_move_t *chassis_move_control_loop);

/*-------------------------------------------------------公共-----------------------------------------------------------*/
//判断优弧劣弧-->只转3508，不转6020，把6020角度控制在0-90度内，可以更好的转换运动方向
void Speed_Toggle(chassis_move_t *chassis_move_control_Speed);

//清除PID
static void PID_CLEAR_ALL(chassis_move_t *chassis_move_data);

//将角度范围控制在 0 - 8191
float Angle_Limit (float angle ,float max);

//将电机转子转向内侧时 修正方向
fp32 Find_min_Angle(int16_t angle1,fp32 angle2);

void position_judge(chassis_move_t *chassis_move_control_loop);

void num_0_task(chassis_move_t *chassis_move_data);

void cast_function(chassis_move_t *chassis_move_control_loop);
uint32_t SYSTICK;
/*----------------------------------------------------------------------TASK---------------------------------------------------------------------*/
void Remote_RESET(chassis_move_t *chassis_move_rc_to_mode);
int8_t first;
void chassis_task(void const * argument)
{
	chassis_move.Auto_init_mode=1;
	chassis_move.Remote_init_mode=1;
	while(1) 
	{
		//底盘控制模式选择
		Chassis_mode_change(&chassis_move);
		Remote_RESET(&chassis_move);
			SYSTICK=HAL_GetTick();
		//模式判断在前，失能底盘在后
		if(toe_is_error(DBUS_TOE))
		{
			CAN_cmd_chassis(0,0,0,0);
			CAN_cmd_gimbal(0,0,0,0);
			if(num==10 && first==0){num=0;first=1;}
			if(num==0 && first==0){num=10;first=1;}
		}
		else
		{
			first=0;
			angle_judge(&chassis_move);
			CAN_cmd_gimbal(chassis_move.chassis_6020_speed_pid[0].out, chassis_move.chassis_6020_speed_pid[1].out,
														chassis_move.chassis_6020_speed_pid[2].out, chassis_move.chassis_6020_speed_pid[3].out);		
			if(chassis_move.angle_ready)
			{
				CAN_cmd_chassis(chassis_move.chassis_3508_speed_pid[0].out, chassis_move.chassis_3508_speed_pid[1].out,
												chassis_move.chassis_3508_speed_pid[2].out, chassis_move.chassis_3508_speed_pid[3].out);
			}
		}
		osDelay(2);//控制频率为1khz，与can接收中断频率一致
	}
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------*/

void Remote_RESET(chassis_move_t *chassis_move_rc_to_mode)
{
	static int32_t now_time,last_time,abc;
	if(chassis_move_rc_to_mode->chassis_RC->rc.ch[4]>600 && !abc)
	{
		last_time=xTaskGetTickCount();
		abc=1;
	}
	now_time=xTaskGetTickCount();
	if(now_time-last_time>1000 && chassis_move_rc_to_mode->chassis_RC->rc.ch[4]>600)
	{
		HAL_NVIC_SystemReset();
		abc = 0;
	}
}

/*****************************************************模式选择函数******************************************/
int mode_change=0;
//自动模式和遥控模式选择
void Chassis_mode_change(chassis_move_t *chassis_move_rc_to_mode)
{
	//左边
	switch(chassis_move_rc_to_mode->chassis_RC->rc.s[1])
	{
		//上拨
		case 1:	PID_CLEAR_ALL(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->my_chassis_mode = Chassis_Remote;
						osThreadSuspend(LED_FLOW_TASKHandle);												
						chassis_move_rc_to_mode->Auto_init_mode = 1;
						if(chassis_move_rc_to_mode->Remote_init_mode)
						remote_control_chassis_init(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->Remote_init_mode = 0;											
						chassis_remote_control_loop(chassis_move_rc_to_mode);			
						switch(chassis_move_rc_to_mode->chassis_RC->rc.s[0])
						{
							case 3:chassis_move_rc_to_mode->my_remote_mode = chassis_top_move;break;
							default :chassis_move_rc_to_mode->my_remote_mode =CHASSIS_ZERO_FORCE;
						}
		break;
						
		//下面		
		case 2:	
			      PID_CLEAR_ALL(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->my_chassis_mode = Chassis_Automatic;													
						osThreadResume(LED_FLOW_TASKHandle);																							
						chassis_move_rc_to_mode->Remote_init_mode = 1;
						if(chassis_move_rc_to_mode->Auto_init_mode)     
							automatic_control_chassis_init(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->Auto_init_mode = 0;		
						cast_function(chassis_move_rc_to_mode);	
		break;
		
		//中间				
		case 3:			
			PID_CLEAR_ALL(chassis_move_rc_to_mode);
			CAN_cmd_chassis(0,0,0,0);
			CAN_cmd_gimbal(0,0,0,0);
			break;
	}
					if(num==1 && mode_change==0)
	{
		mode_change=1;
		PID_CLEAR_ALL(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->my_chassis_mode = Chassis_Remote;
						osThreadSuspend(LED_FLOW_TASKHandle);												
						chassis_move_rc_to_mode->Auto_init_mode = 1;
						if(chassis_move_rc_to_mode->Remote_init_mode)
						remote_control_chassis_init(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->Remote_init_mode = 0;											
						chassis_remote_control_loop(chassis_move_rc_to_mode);			
						switch(chassis_move_rc_to_mode->chassis_RC->rc.s[0])
						{
							case 3:chassis_move_rc_to_mode->my_remote_mode = chassis_top_move;break;
							default :chassis_move_rc_to_mode->my_remote_mode =CHASSIS_ZERO_FORCE;
						}
	}
						if(num==2 && mode_change==1)
	{
		mode_change=2;
		PID_CLEAR_ALL(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->my_chassis_mode = Chassis_Remote;
						osThreadSuspend(LED_FLOW_TASKHandle);												
						chassis_move_rc_to_mode->Auto_init_mode = 1;
						if(chassis_move_rc_to_mode->Remote_init_mode)
						remote_control_chassis_init(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->Remote_init_mode = 0;											
						chassis_remote_control_loop(chassis_move_rc_to_mode);			
						switch(chassis_move_rc_to_mode->chassis_RC->rc.s[0])
						{
							case 3:chassis_move_rc_to_mode->my_remote_mode = chassis_top_move;break;
							default :chassis_move_rc_to_mode->my_remote_mode =CHASSIS_ZERO_FORCE;
						}
	}
						if(num==13 && mode_change==0)
	{
		mode_change=1;
		PID_CLEAR_ALL(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->my_chassis_mode = Chassis_Remote;
						osThreadSuspend(LED_FLOW_TASKHandle);												
						chassis_move_rc_to_mode->Auto_init_mode = 1;
						if(chassis_move_rc_to_mode->Remote_init_mode)
						remote_control_chassis_init(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->Remote_init_mode = 0;											
						chassis_remote_control_loop(chassis_move_rc_to_mode);			
						switch(chassis_move_rc_to_mode->chassis_RC->rc.s[0])
						{
							case 3:chassis_move_rc_to_mode->my_remote_mode = chassis_top_move;break;
							default :chassis_move_rc_to_mode->my_remote_mode =CHASSIS_ZERO_FORCE;
						}
	}
							if(num==17 && mode_change==1)
	{
		mode_change=2;
		PID_CLEAR_ALL(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->my_chassis_mode = Chassis_Remote;
						osThreadSuspend(LED_FLOW_TASKHandle);												
						chassis_move_rc_to_mode->Auto_init_mode = 1;
						if(chassis_move_rc_to_mode->Remote_init_mode)
						remote_control_chassis_init(chassis_move_rc_to_mode);
						chassis_move_rc_to_mode->Remote_init_mode = 0;											
						chassis_remote_control_loop(chassis_move_rc_to_mode);			
						switch(chassis_move_rc_to_mode->chassis_RC->rc.s[0])
						{
							case 3:chassis_move_rc_to_mode->my_remote_mode = chassis_top_move;break;
							default :chassis_move_rc_to_mode->my_remote_mode =CHASSIS_ZERO_FORCE;
						}
	}
}
/*-----------------------------------------------------------------------------------------------------------------------*/
/**************************************************自动模式相关函数********************************************************/
/*-----------------------------------------------------------------------------------------------------------------------*/

//初始化函数
static void automatic_control_chassis_init(chassis_move_t* chassis_move_init)
{
	uint8_t i=0;
	static float SPEED_3508[3]={AUTO_3508_SPEED_PID_KP,AUTO_3508_SPEED_PID_KI,AUTO_3508_SPEED_PID_KD};//PID参数设置
	static float ANGLE_3508[3]={AUTO_3508_ANGLE_PID_KP,AUTO_3508_ANGLE_PID_KI,AUTO_3508_ANGLE_PID_KD};//PID参数设置
	static float ANGLE_6020[3]={M6020_MOTOR_ANGLE_PID_KP_AUTO,M6020_MOTOR_ANGLE_PID_KI_AUTO,M6020_MOTOR_ANGLE_PID_KD_AUTO};//PID参数设置
	static float SPEED_6020[3]={M6020_MOTOR_SPEED_PID_KP_AUTO,M6020_MOTOR_SPEED_PID_KI_AUTO,M6020_MOTOR_SPEED_PID_KD_AUTO};//PID参数设置
	const static fp32 chassis_3508_order_filter[1] = {CHASSIS_ACCEL_3508_NUM};

	//chassis_move_init->drct=1;
	chassis_move_init->angle_ready=0;
	chassis_move_init->my_auto_mode = ALL_DIRECTION;
	/**初始化3508双环PID 并获取电机数据**/
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);//获取底盘3508的数据，接收电机的反馈结构体
		//位置环
		PID_init(&chassis_move_init->chassis_3508_angle_pid[i],PID_POSITION,ANGLE_3508,AUTO_3508_ANGLE_PID_MAX_OUT,AUTO_3508_ANGLE_PID_MAX_IOUT);//初始化底盘PID
		//速度环
		PID_init(&chassis_move_init->chassis_3508_speed_pid[i],PID_POSITION,SPEED_3508,AUTO_3508_SPEED_PID_MAX_OUT,AUTO_3508_SPEED_PID_MAX_IOUT);//初始化底盘PID
		chassis_move_init->wheel_speed[i]=0.0f;
	}
	//初始化6020速度和角度PID并获取电机数据
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i+4].chassis_motor_measure = get_gimbal_motor_measure_point(i);//获取航向电机的数据，接收电机的反馈结构体
		PID_init(&chassis_move_init->chassis_6020_speed_pid[i],PID_POSITION,SPEED_6020,M6020_MOTOR_SPEED_PID_MAX_OUT_AUTO,M6020_MOTOR_SPEED_PID_MAX_IOUT_AUTO);//初始化速度PID
		PID_init(&chassis_move_init->chassis_6020_angle_pid[i],PID_POSITION,ANGLE_6020,M6020_MOTOR_ANGLE_PID_MAX_OUT_AUTO,M6020_MOTOR_ANGLE_PID_MAX_IOUT_AUTO);//初始化角度PID
		chassis_move_init->AGV_wheel_Angle[i]=0.0f;
	}
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_3508, CHASSIS_CONTROL_TIME_3508, chassis_3508_order_filter);
}

void AUTO_AGV_Angle_calc(chassis_speed_t *chassis_speed)
{
static fp64 atan_angle[4];
	static fp32 AGV_wheel_Angle_last[4];
	switch(chassis_move.my_remote_mode)
	{
		//右中
		case chassis_top_move:
					atan_angle[0]=atan2(chassis_speed->Vy - chassis_speed->Vw*0.707f,chassis_speed->Vx + chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[1]=atan2(chassis_speed->Vy - chassis_speed->Vw*0.707f,chassis_speed->Vx - chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[2]=atan2(chassis_speed->Vy + chassis_speed->Vw*0.707f,chassis_speed->Vx - chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[3]=atan2(chassis_speed->Vy + chassis_speed->Vw*0.707f,chassis_speed->Vx + chassis_speed->Vw*0.707f)/PI*180.0;
					
		

					// 将一圈360°转换成编码值的一圈0-8191 -> 角度 * 8191 / 360 最终转换为需要转动的角度对应的编码值，再加上偏置角度,最终得到目标编码值
					chassis_move.AGV_wheel_Angle[0]=	Angle_Limit(GIM_AUTO_ECD_1 + (fp32)(atan_angle[0] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[1]=	Angle_Limit(GIM_AUTO_ECD_2 + (fp32)(atan_angle[1] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[2]=	Angle_Limit(GIM_AUTO_ECD_3 + (fp32)(atan_angle[2] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[3]=	Angle_Limit(GIM_AUTO_ECD_4 + (fp32)(atan_angle[3] * 22.75f),ECD_RANGE);
					
//					Speed_Toggle(&chassis_move);
					
					if(chassis_move.vx_set_channel == 0 && chassis_move.vy_set_channel == 0 && chassis_move.vz_set_channel == 0)//摇杆回中时，保持6020角度
					{
						for(int i=0;i<4;i++)//memcpy狗都不用
						chassis_move.AGV_wheel_Angle[i] = AGV_wheel_Angle_last[i];
					}
					else
					{
						for(int i=0;i<4;i++)
						{
							AGV_wheel_Angle_last[i]=chassis_move.AGV_wheel_Angle[i];
						}
					}
					for(int i=0;i<4;i++)//减少手抖误差 和回中误差
					{
						if(fabs(chassis_move.AGV_wheel_Angle[i]-AGV_wheel_Angle_last[i])<22)//小于1度维持原值
						{
							chassis_move.AGV_wheel_Angle[i]=AGV_wheel_Angle_last[i];
						}
					}	
				break;
	}
}


int position_judge_flag=0;

//自动模式下舵轮3508解算
static void Auto_chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop)
{
	chassis_move_control_loop->derta_length = Chassis_Me_To_Target_Distance(chassis_move_control_loop);
			for(int i=0;i<4;i++)
		{
			chassis_move_control_loop->wheel_speed[i] = chassis_move_control_loop->drct*chassis_move_control_loop->derta_length;

		}
}

//自动模式下舵轮6020解算
static void Auto_chassis_AGV_wheel_angle(chassis_move_t *chassis_move_control_loop)
{
	Polar_To_Stright(chassis_move_control_loop,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);
	//6020角度
	chassis_move_control_loop->derta_ecd = chassis_move_control_loop->target_A*22.75f;

			chassis_move_control_loop->AGV_wheel_Angle[0] = Angle_Limit(chassis_move_control_loop->derta_ecd+GIM_AUTO_ECD_1+final_yaw*22.75,ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[1] = Angle_Limit(chassis_move_control_loop->derta_ecd+GIM_AUTO_ECD_2+final_yaw*22.75,ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[2] = Angle_Limit(chassis_move_control_loop->derta_ecd+GIM_AUTO_ECD_3+final_yaw*22.75,ECD_RANGE);
			chassis_move_control_loop->AGV_wheel_Angle[3] = Angle_Limit(chassis_move_control_loop->derta_ecd+GIM_AUTO_ECD_4+final_yaw*22.75,ECD_RANGE);

	Speed_Toggle(chassis_move_control_loop);
}

//自动模式pid计算
static void Auto_pid_caculate(chassis_move_t *chassis_move_control_loop)
{
	uint8_t i=0;

	//计算6020双环
	for(i=0;i<4;i++)
	{
			//角度环
			PID_Calc_Angle(&chassis_move_control_loop->chassis_6020_angle_pid[i],
				chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->ecd,chassis_move_control_loop->AGV_wheel_Angle[i]);
		if(chassis_move_control_loop->step[0])
	{
		chassis_move_control_loop->chassis_6020_angle_pid[i].out=0;
	}
			//速度环
			PID_calc(&chassis_move_control_loop->chassis_6020_speed_pid[i],
				chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->rpm,
				chassis_move_control_loop->chassis_6020_angle_pid[i].out);
	//计算3508双环
			if((num >= 10 && num <=20 )|| (num==0))limit_speed(chassis_move_control_loop);
			//位置环
		PID_calc(&chassis_move_control_loop->chassis_3508_angle_pid[i],chassis_move_control_loop->wheel_speed[i],0);
			//速度环
		
		if(slow_down)
		{
			chassis_move_control_loop->chassis_3508_angle_pid[i].out=-chassis_move_control_loop->drct*666;
		}
		if(stop_flag)
		{
			chassis_move_control_loop->chassis_3508_angle_pid[i].out=0;
		}

		PID_calc(&chassis_move_control_loop->chassis_3508_speed_pid[i],
				chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->rpm,
				chassis_move_control_loop->chassis_3508_angle_pid[i].out);
//		chassis_move_control_loop->derta_length=Chassis_Me_To_Target_X_Distance(chassis_move_control_loop);
//					if(num==1 && fabs(chassis_move_control_loop->derta_length) >50 && yaw_calibrate_flag)
//	{
//		chassis_move_control_loop->chassis_3508_speed_pid[i].out=fabs(chassis_move_control_loop->chassis_3508_speed_pid[i].out)/chassis_move_control_loop->chassis_3508_speed_pid[i].out*12000;
//	}
//				if(num==13 && chassis_move_control_loop->derta_length >60 && chassis_move_control_loop->derta_length <95 && yaw_calibrate_flag)
//	{
//		chassis_move_control_loop->chassis_3508_speed_pid[i].out=fabs(chassis_move_control_loop->chassis_3508_speed_pid[i].out)/chassis_move_control_loop->chassis_3508_speed_pid[i].out*10000;
//	}
}
}

/*-----------------------------------------------------------------------------------------------------------------------*/
/***************************************************遥控器模式相关函数*****************************************************/
/*-----------------------------------------------------------------------------------------------------------------------*/

//初始化
static void remote_control_chassis_init(chassis_move_t* chassis_move_init)
{
	uint8_t i=0;
	
	static float PID_SPEED_3508[3]={M3508_MOTOR_SPEED_PID_KP,M3508_MOTOR_SPEED_PID_KI,M3508_MOTOR_SPEED_PID_KD};//PID参数设置
	static float PID_ANGLE_6020[3]={M6020_MOTOR_ANGLE_PID_KP,M6020_MOTOR_ANGLE_PID_KI,M6020_MOTOR_ANGLE_PID_KD};//PID参数设置
	static float PID_SPEED_6020[3]={M6020_MOTOR_SPEED_PID_KP,M6020_MOTOR_SPEED_PID_KI,M6020_MOTOR_SPEED_PID_KD};//PID参数设置
	
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  const static fp32 chassis_z_order_filter[1] = {CHASSIS_ACCEL_Z_NUM};
	
	chassis_move_init->drct=1;
	chassis_move_init->angle_ready=0;
//	chassis_move_init->yaw_init = imu.yaw;
//	chassis_move.chassis_RC=get_remote_control_point();
	
	//初始化3508速度PID 并获取电机数据
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);//获取底盘3508的数据，接收电机的反馈结构体		
		PID_init(&chassis_move_init->chassis_3508_speed_pid[i],PID_POSITION,PID_SPEED_3508,M3508_MOTOR_SPEED_PID_MAX_OUT,M3508_MOTOR_SPEED_PID_MAX_IOUT);//初始化底盘PID
		chassis_move_init->wheel_speed[i]=0.0f;
	}
	
	//初始化6020速度和角度PID并获取电机数据
	for(i=0;i<4;i++)
	{
		chassis_move_init->motor_chassis[i+4].chassis_motor_measure = get_gimbal_motor_measure_point(i);//获取航向电机的数据，接收电机的反馈结构体
		PID_init(&chassis_move_init->chassis_6020_speed_pid[i],PID_POSITION,PID_SPEED_6020,M6020_MOTOR_SPEED_PID_MAX_OUT,M6020_MOTOR_SPEED_PID_MAX_IOUT);//初始化速度PID
		PID_init(&chassis_move_init->chassis_6020_angle_pid[i],PID_POSITION,PID_ANGLE_6020,M6020_MOTOR_ANGLE_PID_MAX_OUT,M6020_MOTOR_ANGLE_PID_MAX_IOUT);//初始化角度PID
		chassis_move_init->AGV_wheel_Angle[i]=0.0f;
	}
	//用一阶滤波
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vz, CHASSIS_CONTROL_TIME_Z, chassis_z_order_filter);
	
}


static void chassis_remote_control_loop(chassis_move_t *chassis_move_control_loop)
{
	//遥控器数值获取加死区限制和平均滤波
	Chassis_rc_to_control_vector(chassis_move_control_loop);
	
	//底盘速度获取
	Chassis_AGV_wheel_speed(chassis_move_control_loop);
	
	//转换为机器人坐标系
	Robot_coordinate(&chassis_move_control_loop->absolute_chassis_speed,final_yaw+90);
	
	//pid计算
	Pid_caculate(chassis_move_control_loop);

	//等待6020角度转到位在转3508
	angle_judge(chassis_move_control_loop);
}
/**
  * @brief  计算底盘驱动电机6020的目标角度
	* @param  chassis_speed 转化为机器人坐标后的速度
  * @retval 
  * @attention  AGV_wheel_Angle 6020目标角度
  */
void AGV_Angle_calc(chassis_speed_t *chassis_speed)
{
	static fp64 atan_angle[4];
	static fp32 AGV_wheel_Angle_last[4];
	switch(chassis_move.my_remote_mode)
	{
		//右中
		case chassis_top_move:
					atan_angle[0]=atan2(chassis_speed->Vy - chassis_speed->Vw*0.707f,chassis_speed->Vx + chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[1]=atan2(chassis_speed->Vy - chassis_speed->Vw*0.707f,chassis_speed->Vx - chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[2]=atan2(chassis_speed->Vy + chassis_speed->Vw*0.707f,chassis_speed->Vx - chassis_speed->Vw*0.707f)/PI*180.0;
					atan_angle[3]=atan2(chassis_speed->Vy + chassis_speed->Vw*0.707f,chassis_speed->Vx + chassis_speed->Vw*0.707f)/PI*180.0;
					
		

					// 将一圈360°转换成编码值的一圈0-8191 -> 角度 * 8191 / 360 最终转换为需要转动的角度对应的编码值，再加上偏置角度,最终得到目标编码值
					chassis_move.AGV_wheel_Angle[0]=	Angle_Limit(GIM_AUTO_ECD_1 + (fp32)(atan_angle[0] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[1]=	Angle_Limit(GIM_AUTO_ECD_2 + (fp32)(atan_angle[1] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[2]=	Angle_Limit(GIM_AUTO_ECD_3 + (fp32)(atan_angle[2] * 22.75f),ECD_RANGE);
					chassis_move.AGV_wheel_Angle[3]=	Angle_Limit(GIM_AUTO_ECD_4 + (fp32)(atan_angle[3] * 22.75f),ECD_RANGE);
					
					Speed_Toggle(&chassis_move);
					
					if(chassis_move.vx_set_channel == 0 && chassis_move.vy_set_channel == 0 && chassis_move.vz_set_channel == 0)//摇杆回中时，保持6020角度
					{
						for(int i=0;i<4;i++)//memcpy狗都不用
						chassis_move.AGV_wheel_Angle[i] = AGV_wheel_Angle_last[i];
					}
					else
					{
						for(int i=0;i<4;i++)
						{
							AGV_wheel_Angle_last[i]=chassis_move.AGV_wheel_Angle[i];
						}
					}
					for(int i=0;i<4;i++)//减少手抖误差 和回中误差
					{
						if(fabs(chassis_move.AGV_wheel_Angle[i]-AGV_wheel_Angle_last[i])<22)//小于1度维持原值
						{
							chassis_move.AGV_wheel_Angle[i]=AGV_wheel_Angle_last[i];
						}
					}	
				break;
	}
}
/**
  * @brief  计算底盘驱动电机3508的目标速度
	* @param  chassis_speed 转化为机器人坐标后的速度
  * @retval 
  * @attention	wheel_speed 3508目标速度
  */
void AGV_Speed_calc(chassis_speed_t *chassis_speed)
{

		chassis_move.wheel_speed[0] = chassis_move.drct*sqrt(pow((chassis_speed->Vy - chassis_speed->Vw*0.707f),2)+pow(chassis_speed->Vx + chassis_speed->Vw*0.707f,2));
    chassis_move.wheel_speed[1] = chassis_move.drct*sqrt(pow((chassis_speed->Vy - chassis_speed->Vw*0.707f),2)+pow(chassis_speed->Vx - chassis_speed->Vw*0.707f,2));
	  chassis_move.wheel_speed[2] = chassis_move.drct*sqrt(pow((chassis_speed->Vy + chassis_speed->Vw*0.707f),2)+pow(chassis_speed->Vx - chassis_speed->Vw*0.707f,2));
	  chassis_move.wheel_speed[3] = chassis_move.drct*sqrt(pow((chassis_speed->Vy + chassis_speed->Vw*0.707f),2)+pow(chassis_speed->Vx + chassis_speed->Vw*0.707f,2));

}
/**
	* @brief  将世界坐标下的Vx,Vy,Vw转换为机器人坐标
  * @param  wrold_speed 世界坐标速度
	* @param 	angle 底盘与云台的yaw_diff
  * @retval 
  * @attention	
  */
void Robot_coordinate(chassis_speed_t * wrold_speed, fp32 angle)
{
   fp32 angle_diff=angle* PI / 180;
   chassis_speed_t temp_speed;
   temp_speed.Vw = wrold_speed->Vw; 
		
	 temp_speed.Vx = wrold_speed->Vx * cos(angle_diff) - wrold_speed->Vy * sin(angle_diff);
	 temp_speed.Vy = wrold_speed->Vx * sin(angle_diff) + wrold_speed->Vy * cos(angle_diff);
	
	 AUTO_AGV_Angle_calc(&temp_speed);//6020
   AGV_Speed_calc(&temp_speed);//3508
}

/**
	* @brief 确定底盘运动速度，世界坐标
	* @param 底盘结构体
  * @retval 
	* @attention	不同模式速度不同
	*/
static void Chassis_AGV_wheel_speed(chassis_move_t *chassis_move_control_loop)
{
	switch(chassis_move_control_loop->my_remote_mode)
	{			
		//右中
		case chassis_top_move:
					//PID_CLEAR_ALL(chassis_move_control_loop);
					chassis_move_control_loop->absolute_chassis_speed.Vw = chassis_move_control_loop->vz_set_channel;
					chassis_move_control_loop->absolute_chassis_speed.Vx = chassis_move_control_loop->vx_set_channel;
					chassis_move_control_loop->absolute_chassis_speed.Vy = chassis_move_control_loop->vy_set_channel;
					
		break;
	}
}





//遥控器数值获取加死区限制
void Chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector)
{
  //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[2], chassis_move_rc_to_vector->vx_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[3], chassis_move_rc_to_vector->vy_channel, CHASSIS_RC_DEADLINE);
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[0], chassis_move_rc_to_vector->vz_channel, CHASSIS_RC_DEADLINE);
		
	//8911/660=13.5015 将速度扩大到额定转速
	chassis_move_rc_to_vector->vx_set_channel = chassis_move_rc_to_vector->vx_channel*13.5015;
	chassis_move_rc_to_vector->vy_set_channel = -chassis_move_rc_to_vector->vy_channel*13.5015;
	chassis_move_rc_to_vector->vz_set_channel = chassis_move_rc_to_vector->vz_channel*13.5015;
	
	//一阶低通滤波代替斜波作为底盘速度输入
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, chassis_move_rc_to_vector->vx_set_channel);
	chassis_move_rc_to_vector->vx_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, chassis_move_rc_to_vector->vy_set_channel);
	chassis_move_rc_to_vector->vy_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
	
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vz, chassis_move_rc_to_vector->vz_set_channel);
	chassis_move_rc_to_vector->vz_set_channel = chassis_move_rc_to_vector->chassis_cmd_slow_set_vz.out;

//	chassis_move_rc_to_vector->vx_set = chassis_move_rc_to_vector->vx_set_channel/19*WHEEL_CIRCUMFERENCE;
//	chassis_move_rc_to_vector->vy_set = chassis_move_rc_to_vector->vy_set_channel/19*WHEEL_CIRCUMFERENCE;
//	chassis_move_rc_to_vector->wz_set = chassis_move_rc_to_vector->vy_set_channel/19*WHEEL_CIRCUMFERENCE/MOTOR_DISTANCE_TO_CENTER;	
}

/**
	* @brief  PID计算
  * @param  底盘结构体
  * @retval 
  * @attention	3508单速度环，6020角度环+速度环
  */
void Pid_caculate(chassis_move_t *chassis_move_control_loop)
{
	for (int i = 0; i < 4; i++)
		{
        PID_calc(&chassis_move_control_loop->chassis_3508_speed_pid[i], 
					chassis_move_control_loop->motor_chassis[i].chassis_motor_measure->rpm,chassis_move_control_loop->wheel_speed[i]);
		}
					//计算6020角度串速度环，速度环做内环，角度环做外环
		for(int i=0;i<4;i++)
		{
				//角度环
				PID_Calc_Ecd(&chassis_move_control_loop->chassis_6020_angle_pid[i],
					chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->ecd,chassis_move_control_loop->AGV_wheel_Angle[i],8191.0f);
				//速度环
				PID_calc(&chassis_move_control_loop->chassis_6020_speed_pid[i],
					chassis_move_control_loop->motor_chassis[i+4].chassis_motor_measure->rpm,
					chassis_move_control_loop->chassis_6020_angle_pid[i].out);
		}
}

/*-----------------------------------------------------------------------------------------------公共函数---------------------------------------------------------------------------------------------------*/

//将电机转子转向内侧时 修正方向
fp32 Find_min_Angle(int16_t angle1,fp32 angle2)
{
	fp32 err;
    err = (fp32)angle1 - angle2;
    if(fabs(err) > 4096)
    {
        err = 8192 - fabs(err);
    }
    return err;
}


//等待6020角度转到位在转3508
void angle_judge(chassis_move_t *chassis_move_control_loop)
{
	if(fabs(chassis_move_control_loop->motor_chassis[4].chassis_motor_measure->ecd - chassis_move_control_loop->AGV_wheel_Angle[0]) < 1000)//当角度偏差为50时使能3508转动
		chassis_move_control_loop->angle_ready=1;
	else
		chassis_move_control_loop->angle_ready=0;
}


//判断优弧劣弧-->只转3508，不转6020，把6020角度控制在0-90度内，可以更好的转换运动方向
void Speed_Toggle(chassis_move_t *chassis_move_control_Speed)
{
		if(fabs(Find_min_Angle(chassis_move_control_Speed->motor_chassis[4].chassis_motor_measure->ecd,chassis_move_control_Speed->AGV_wheel_Angle[0]))>2048)
	{
		for(int i=0;i<4;i++)
		{
			chassis_move_control_Speed->AGV_wheel_Angle[i] += 4096;		
			chassis_move_control_Speed->AGV_wheel_Angle[i]=Angle_Limit(chassis_move_control_Speed->AGV_wheel_Angle[i],8192);
		}
			chassis_move_control_Speed->drct = -1;
	}
	else
			chassis_move_control_Speed->drct=1;
}


//清除PID
static void PID_CLEAR_ALL(chassis_move_t *chassis_move_data)
{
	for(uint8_t i=0;i<4;i++)
	{
		PID_clear(&chassis_move_data->chassis_6020_angle_pid[i]);
		PID_clear(&chassis_move_data->chassis_6020_speed_pid[i]);
		PID_clear(&chassis_move_data->chassis_3508_speed_pid[i]);
	}
}

//初始化uwb 的x和y数据 x：363  y：145   a板yaw为0度
//起始世界坐标为（0，-238） 
/** 														-----机器人跑点思路-----																	**/
/**	 	以地图原点为极坐标原点，由于场上竹子和地标位置已知，所以相当于各个点位的极角已知  			**/
/**	 			点位的极径已知，机器人的初始坐标在y轴的负方向238cm处，即初始极径为238cm  					**/
/**				重要的是把极坐标转换为直角坐标之中，这样得出x，y轴速度以及6020的转向角度					**/
/*******计算当前位置Y坐标与目标位置的Y坐标的差值******/

float Chassis_Me_To_Target_Y_Distance(chassis_move_t *chassis_move_data)
{
	return (chassis_move_data->target_Y-chassis_move_data->uwb_data.y);
}

/*******计算当前位置X坐标与目标位置的X坐标的差值******/
float Chassis_Me_To_Target_X_Distance(chassis_move_t *chassis_move_data)
{
	return (chassis_move_data->target_X-chassis_move_data->uwb_data.x);
}

//yaw轴初始化为零  零逆时针360度
fp32 yaw_error;

int16_t yaw_num;
extern int16_t final_yaw;
//计算所有信号以及yaw轴偏航角度
/*******计算当前位置角度坐标与目标位置的角度坐标的差值******/
int16_t Chassis_Me_To_Target_Angle(chassis_move_t *chassis_move_data)
{
//	if(num==0)
//	{
//		yaw_num=50;return final_yaw-yaw_num;
//	}
	if(num==11)
	{
		return final_yaw+chassis_move_data->target_A+90;
	}
	if(num==20 || num==17)
	{
		return final_yaw+chassis_move_data->target_A+90;
	}
	return final_yaw+chassis_move_data->target_A;
//	}
}

/*********计算当前位置与目标位置的差值*************/
float Chassis_Me_To_Target_Distance(chassis_move_t *chassis_move_data)
{
	return sqrt(pow(Chassis_Me_To_Target_X_Distance(chassis_move_data),2)+pow(Chassis_Me_To_Target_Y_Distance(chassis_move_data),2));
}

/****************极坐标转直角坐标*****************/
/******* 车初始坐标 (238,-90°)----->(0,-238) ****/

void Polar_To_Stright(chassis_move_t *chassis_move_data,float plo_angle,float plo_length)
{
	int8_t left_right=0;  //相对左右标志位
	chassis_move_data->target_X =  (plo_length * cosf(plo_angle /180.f * PI));//X   
	chassis_move_data->target_Y =  (plo_length * sinf(plo_angle /180.f * PI));//Y  
	chassis_move_data->derta_X = Chassis_Me_To_Target_X_Distance(chassis_move_data);
	chassis_move_data->derta_Y = Chassis_Me_To_Target_Y_Distance(chassis_move_data);

	if(chassis_move_data->derta_Y > 0 && chassis_move_data->derta_X < 0) //相对车为原点 第二象限
	{
		left_right=-1; //左侧为负
		chassis_move_data->derta_Y = fabs(chassis_move_data->derta_Y);
		chassis_move_data->derta_X = fabs(chassis_move_data->derta_X);
		chassis_move_data->target_A =  atan2(chassis_move_data->derta_Y ,chassis_move_data->derta_X)/PI*180;//相对于车的角度
		chassis_move_data->target_A=left_right*(90.0f-chassis_move_data->target_A);
	}		
	else if(chassis_move_data->derta_Y > 0 && chassis_move_data->derta_X > 0)//第一象限
	{
		left_right=1; //右侧为正
		chassis_move_data->derta_Y = fabs(chassis_move_data->derta_Y);
		chassis_move_data->derta_X = fabs(chassis_move_data->derta_X);
		chassis_move_data->target_A =  atan2(chassis_move_data->derta_Y ,chassis_move_data->derta_X)/PI*180;//相对于车的角度
		chassis_move_data->target_A=left_right*(90.0f-chassis_move_data->target_A);
	}
	else if(chassis_move_data->derta_Y < 0 && chassis_move_data->derta_X < 0)//第三象限
	{
		left_right=-1; //左侧为负
		chassis_move_data->derta_Y = fabs(chassis_move_data->derta_Y);
		chassis_move_data->derta_X = fabs(chassis_move_data->derta_X);
		chassis_move_data->target_A =  atan2(chassis_move_data->derta_Y ,chassis_move_data->derta_X)/PI*180;//相对于车的角度
		chassis_move_data->target_A=left_right*(90.0f+chassis_move_data->target_A);
	}
	else if(chassis_move_data->derta_Y < 0 && chassis_move_data->derta_X > 0)//第四象限
	{
		left_right=1; //右侧为正
		chassis_move_data->derta_Y = fabs(chassis_move_data->derta_Y);
		chassis_move_data->derta_X = fabs(chassis_move_data->derta_X);
		chassis_move_data->target_A =  atan2(chassis_move_data->derta_Y ,chassis_move_data->derta_X)/PI*180;//相对于车的角度
		chassis_move_data->target_A=left_right*(90.0f+chassis_move_data->target_A);
	}
}

void stop_3508(chassis_move_t *chassis_move_control_loop)
{
	//急刹需要控制角度环输出为零，也就是让转速趋近于零，根据当前rpm控制而不是直接基于电流值为零
		chassis_move.wheel_speed[0]=0;
		chassis_move.wheel_speed[1]=0;
		chassis_move.wheel_speed[2]=0;
		chassis_move.wheel_speed[3]=0;
}

void Top_6020_enforce(chassis_move_t *chassis_move_control_loop)
{
		chassis_move_control_loop->AGV_wheel_Angle[0]=	GIM_TOP_ECD1;
		chassis_move_control_loop->AGV_wheel_Angle[1]=	GIM_TOP_ECD2;
		chassis_move_control_loop->AGV_wheel_Angle[2]=	GIM_TOP_ECD3;
		chassis_move_control_loop->AGV_wheel_Angle[3]=	GIM_TOP_ECD4;
}

void speed_3508_enforce(chassis_move_t *chassis_move_control_loop,float speed)
{
		chassis_move_control_loop->wheel_speed[0]=speed;
		chassis_move_control_loop->wheel_speed[1]=speed;
		chassis_move_control_loop->wheel_speed[2]=speed;
		chassis_move_control_loop->wheel_speed[3]=speed;
}

//top急停
void top_stop(chassis_move_t *chassis_move_control_loop)
{
	stop_3508(chassis_move_control_loop);
	Top_6020_enforce(chassis_move_control_loop);
}

void yaw_calibrate(chassis_move_t *chassis_move_control_loop)
{
	//计算yaw轴偏差
	yaw_error=Chassis_Me_To_Target_Angle(chassis_move_control_loop);
	if(yaw_error>180 && yaw_error<360)
	{
		yaw_error=-(360-yaw_error);
	}
	if(yaw_error<-180 && yaw_error>-360)
	{
		yaw_error=180+(yaw_error+180);
	}	

	//给定yaw正负3度的偏差
	if(fabs(yaw_error)>2)
	{		//6020转到小陀螺
			yaw_calibrate_flag=0;
			Top_6020_enforce(chassis_move_control_loop);
		if(num==5)
		{
			speed_3508_enforce(chassis_move_control_loop,-fabs(yaw_error)/yaw_error*120);
		}
		else
		{
			speed_3508_enforce(chassis_move_control_loop,-fabs(yaw_error)/yaw_error*100);
		}
	}
	else
	{
		yaw_calibrate_flag=1;
	}
				//pid计算
				Auto_pid_caculate(chassis_move_control_loop);
}

void position_judge(chassis_move_t *chassis_move_control_loop)
{
	chassis_move_control_loop->derta_length = Chassis_Me_To_Target_Distance(chassis_move_control_loop);
	if(num==0 )
	{
		if(chassis_move_control_loop->derta_length < 40)position_judge_flag=1;
	}
	else if(num==1 )
	{
			if(chassis_move_control_loop->derta_length <30)position_judge_flag=1;
	}
	else if( num ==3)
	{
			if(chassis_move_control_loop->derta_length <40)position_judge_flag=1;
	}	
	else if( num ==7)
	{
			if(chassis_move_control_loop->derta_length <45)position_judge_flag=1;
	}	
		else if( num ==9)
	{
			if(chassis_move_control_loop->derta_length <20)position_judge_flag=1;
	}	
	//转点
	else if(num == 2 || num == 4 || num == 6 || num == 8)
	{
		if(chassis_move_control_loop->derta_length <40)position_judge_flag=1;

	}
	else if(num==30)
	{
		if(chassis_move_control_loop->derta_length <55)position_judge_flag=1;
	}
	else
	{	
		if(chassis_move_control_loop->derta_length < 45)
		{
			position_judge_flag=1;
		}
	
		
		else
		{
			position_judge_flag=0;
		}
	}
}
void clear_step(chassis_move_t *chassis_move_data)
{
	for(int i=0;i<5;i++)
	{
		chassis_move_data->step[i]=0;
	}
}

#define Paw_1_catch 1
#define Paw_1_loose 2
#define Paw_2_catch 3
#define Paw_2_loose 4
#define Paw_3_catch 5
#define Paw_3_loose 6

#define average_min_speed 180 
#define yaw_calibrate_speed 100
#define middle_get_speed 130
#define track_speed 180

//时间时间
uint64_t current_time;
uint64_t last_time;
uint8_t time_save;

void speed_limit(chassis_move_t *chassis_move_control_loop,int min_speed)
{
	for(int i=0;i<4;i++)
	{
			if(chassis_move_control_loop->wheel_speed[i]<0 && chassis_move_control_loop->wheel_speed[i]>-min_speed)
			{
				 chassis_move_control_loop->wheel_speed[i]=-min_speed;
			}
			if(chassis_move_control_loop->wheel_speed[i]>0 && chassis_move_control_loop->wheel_speed[i]<min_speed)
			{
				chassis_move_control_loop->wheel_speed[i]=min_speed;
			}
		}
}

//限制最低速度
void limit_speed(chassis_move_t *chassis_move_control_loop)
{
	if(!stop_flag)
	{
		if(num==0)speed_limit(chassis_move_control_loop,yaw_calibrate_speed-10);
		if(num==11)speed_limit(chassis_move_control_loop,yaw_calibrate_speed+10);
		if(num==10)speed_limit(chassis_move_control_loop,yaw_calibrate_speed-30);
		else if(num==1 && num_1_flag == 0)speed_limit(chassis_move_control_loop,yaw_calibrate_speed);
		else if(num==1 && num_1_flag == 1)speed_limit(chassis_move_control_loop,average_min_speed);		
		else if(num==3 && num_3_flag == 0)speed_limit(chassis_move_control_loop,yaw_calibrate_speed);
		else if(num==3 && num_3_flag == 1)speed_limit(chassis_move_control_loop,average_min_speed);
		else if(num == 2 || num == 4 /*|| num == 6*/ || num == 8)speed_limit(chassis_move_control_loop,220);
		else if(num ==7 && num_7_flag ==0)speed_limit(chassis_move_control_loop,120);
		else if(num ==7 && num_7_flag ==1)speed_limit(chassis_move_control_loop,100);
		else if(num ==9 && num_9_flag ==0)speed_limit(chassis_move_control_loop,50);
		else if(num ==9 && num_9_flag ==1)speed_limit(chassis_move_control_loop,average_min_speed);
		else if(num == 13 && num == 15 && num == 17 && num ==18 && num == 19 )speed_limit(chassis_move_control_loop,150);
		if(over)speed_limit(chassis_move_control_loop,track_speed);
		else speed_limit(chassis_move_control_loop,average_min_speed);
	for(int i=0;i<4;i++)
	{
			if(num==10) 
			{
				if(chassis_move_control_loop->wheel_speed[i]<-120)
			{
				 chassis_move_control_loop->wheel_speed[i]=-120;
			}
			if(chassis_move_control_loop->wheel_speed[i]>120)
			{
				chassis_move_control_loop->wheel_speed[i]=120;
			}
			}
		}
	}
}

void put_first_task(chassis_move_t *chassis_move_data,int Paw_catch,int Paw_loose,int GD)
{
		Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	//校准完成之后
	yaw_calibrate_first(chassis_move_data);
	current_time= HAL_GetTick();
	position_judge(chassis_move_data);
		if(position_judge_flag)
		{
		slow_down=1;
		}
	if(chassis_move_data->GD_flag[GD] && !chassis_move_data->step[0])
	{
	if(!time_save)
		{
		time_save=1;
		last_time=current_time;
		}
	}
		if(current_time-last_time>2500 && chassis_move_data->GD_flag[GD]){
		chassis_move_data->lift_circle_number=put_bamboo;
		chassis_move_data->step[0]=1;
		stop_3508(chassis_move_data);
		stop_flag=1;}
	if(chassis_move_data->step[0])
	{
		stop_3508(chassis_move_data);
		if(chassis_move_data->lift_over)
		{
		if(current_time-last_time>3000)chassis_move_data->Paw_flag=Paw_loose;
		if(current_time-last_time>3500){chassis_move_data->lift_circle_number=second;chassis_move_data->step[1]=1;stop_3508(chassis_move_data);}
		if(chassis_move_data->lift_over && chassis_move_data->step[1])
		{
			if(current_time-last_time>3900){chassis_move_data->Paw_flag=Paw_catch;chassis_move_data->step[2]=1;stop_3508(chassis_move_data);}
		}
		if(current_time-last_time>4400 && chassis_move_data->step[2])
		{
			chassis_move_data->lift_circle_number=ceiling;
			if(chassis_move_data->lift_over)
			{
				chassis_move_data->step[3]=1;
				if(current_time-last_time>4900)
				{
				num++; //完成
				clear_step(chassis_move_data);  //清除所有标志位
				yaw_calibrate_flag=0;
				time_save=0;
				stop_flag=0;
				slow_down=0;
				position_judge_flag=0;
				}
			}
		}
	}
	}
}

void num_1_task(chassis_move_t *chassis_move_data,int Paw_catch,int Paw_loose,int GD)
{
		Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	//校准完成之后
	yaw_calibrate_first(chassis_move_data);
	current_time= HAL_GetTick();
	position_judge(chassis_move_data);
		if(position_judge_flag)
		{
		slow_down=1;
		}
	if(chassis_move_data->GD_flag[GD] && !chassis_move_data->step[0])
	{
	if(!time_save)
		{
		time_save=1;
		last_time=current_time;
		}
	}
		if(current_time-last_time>4000 && chassis_move_data->GD_flag[GD]){
		chassis_move_data->lift_circle_number=put_bamboo;
		chassis_move_data->step[0]=1;
		stop_3508(chassis_move_data);
		stop_flag=1;}
	if(chassis_move_data->step[0])
	{
		stop_3508(chassis_move_data);
		if(chassis_move_data->lift_over)
		{
		if(current_time-last_time>4500)chassis_move_data->Paw_flag=Paw_loose;
		if(current_time-last_time>5000){chassis_move_data->lift_circle_number=second;chassis_move_data->step[1]=1;stop_3508(chassis_move_data);}
		if(chassis_move_data->lift_over && chassis_move_data->step[1])
		{
			if(current_time-last_time>5500){chassis_move_data->Paw_flag=Paw_catch;chassis_move_data->step[2]=1;stop_3508(chassis_move_data);}
		}
		if(current_time-last_time>6000 && chassis_move_data->step[2])
		{
			chassis_move_data->lift_circle_number=ceiling;
			if(chassis_move_data->lift_over)
			{
				chassis_move_data->step[3]=1;
				if(current_time-last_time>6500)
				{
				num++; //完成
				clear_step(chassis_move_data);  //清除所有标志位
				yaw_calibrate_flag=0;
				time_save=0;
				stop_flag=0;
				slow_down=0;
				position_judge_flag=0;
				}
			}
		}
	}
	}
}

//放置第二个竹子之后
void put_second_task(chassis_move_t *chassis_move_data,int Paw_loose,int GD)
{
	//校准完成之后
	yaw_calibrate_first(chassis_move_data);
	current_time= HAL_GetTick();
	position_judge(chassis_move_data);
			if(position_judge_flag)
		{
		slow_down=1;
		}
	if(chassis_move_data->GD_flag[GD] && !chassis_move_data->step[0])
	{

		if(!time_save)
		{
		time_save=1;
		last_time=HAL_GetTick();
		}
	if(current_time-last_time>1000){stop_3508(chassis_move_data);chassis_move_data->lift_circle_number=put_bamboo;chassis_move_data->step[0]=1;stop_flag=1;}
	}
	
	if(chassis_move_data->step[0])
	{
		stop_3508(chassis_move_data);
		if(chassis_move_data->lift_over)
		{
		if(current_time-last_time>2000)chassis_move_data->Paw_flag=Paw_loose; chassis_move_data->step[1]=1;stop_3508(chassis_move_data);
		}
	}
	
	if(current_time-last_time>2500 && chassis_move_data->step[1])
	{
		num++; //完成
		clear_step(chassis_move_data);  //清除所有标志位
		yaw_calibrate_flag=0;
		time_save=0;
		stop_flag=0;
		slow_down=0;
		position_judge_flag=0;
		if(num==19)
		{
			over=1;
		}
	}
}

//void num_0_task(chassis_move_t *chassis_move_data)
//{
//	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
//	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
//	yaw_calibrate(chassis_move_data); 
//	
//	else{
//	//校准完成之后
//	stop_3508(chassis_move_data);
//	Auto_chassis_AGV_wheel_speed(chassis_move_data);
//	Auto_chassis_AGV_wheel_angle(chassis_move_data);
//	position_judge(chassis_move_data);
//		if(position_judge_flag)
//		{
//			stop_3508(chassis_move_data);
//			stop_flag=1;
//		}
//	if(chassis_move.GD_flag[0] && !time_save){time_save=1;last_time = HAL_GetTick();}
//	current_time= HAL_GetTick();
//	if(chassis_move.GD_flag[0])
//	{
//	if(current_time-last_time > 600){chassis_move_data->Paw_flag=Paw_2_catch;}
//}
//	if(chassis_move_data->Paw_flag==Paw_2_catch)
//	{
//	if(current_time-last_time > 1300){chassis_move_data->lift_circle_number=ceiling;chassis_move_data->step[0]=1;}
//}
//	if( chassis_move_data->step[0])
//	{
//		if(chassis_move_data->lift_over)
//		{
//				num++;
//				clear_step(chassis_move_data);
//				time_save=0;
//				yaw_calibrate_flag=0;
//				stop_flag=0;
//				position_judge_flag=0;

//		}
//	}
//}
//}

void four_yaw_space(chassis_move_t *chassis_move_data)
{
	if(chassis_move_data->target_A < 0 && chassis_move_data->target_A > -90) 
	{
	chassis_move_data->absolute_chassis_speed.Vx = -chassis_move_data->vx_set_channel;
	chassis_move_data->absolute_chassis_speed.Vy = -chassis_move_data->vy_set_channel;
	}
	if(chassis_move_data->target_A < -90 && chassis_move_data->target_A > -180) 
	{
	chassis_move_data->absolute_chassis_speed.Vx = -chassis_move_data->vx_set_channel;
	chassis_move_data->absolute_chassis_speed.Vy = chassis_move_data->vy_set_channel;
	}
	if(chassis_move_data->target_A > 0 && chassis_move_data->target_A < 90) 
	{
	chassis_move_data->absolute_chassis_speed.Vx = chassis_move_data->vx_set_channel;
	chassis_move_data->absolute_chassis_speed.Vy = -chassis_move_data->vy_set_channel;
	}
	if(chassis_move_data->target_A > 90 && chassis_move_data->target_A < 180) 
	{
	chassis_move_data->absolute_chassis_speed.Vx = chassis_move_data->vx_set_channel;
	chassis_move_data->absolute_chassis_speed.Vy = chassis_move_data->vy_set_channel;
	}
}

void yaw_calibrate_first(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	yaw_error=Chassis_Me_To_Target_Angle(chassis_move_data);

		if(yaw_error>180 && yaw_error<360)yaw_error=-(360-yaw_error);
	if(yaw_error<-180 && yaw_error>-360)yaw_error=180+(yaw_error+180);
//		if(num==17){yaw_error+=90;}
	chassis_move_data->vx_set_channel=chassis_move_data->derta_X*2.25f;
	chassis_move_data->vy_set_channel=chassis_move_data->derta_Y*2.25f;
	if(num==1)
	{
	chassis_move_data->vx_set_channel=chassis_move_data->derta_X*2.25f;
	chassis_move_data->vy_set_channel=chassis_move_data->derta_Y*2.25f;
	}
	if(num==3)
	{
	chassis_move_data->vx_set_channel=chassis_move_data->derta_X*2;
	chassis_move_data->vy_set_channel=chassis_move_data->derta_Y*2;
	}
	if(num==12)
	{
	chassis_move_data->vx_set_channel=chassis_move_data->derta_X*2;
	chassis_move_data->vy_set_channel=chassis_move_data->derta_Y*2;
	}
	if(num==2)
	{
	chassis_move_data->vx_set_channel=chassis_move_data->derta_X*1.75f;
	chassis_move_data->vy_set_channel=chassis_move_data->derta_Y*1.75f;
	}
	if(num==17)
	{
	chassis_move_data->vx_set_channel=chassis_move_data->derta_X*1;
	chassis_move_data->vy_set_channel=chassis_move_data->derta_Y*1;
	}
	if(num==0 || num==1 || num==11 || num==10 || num==3)  //取地上竹子
	chassis_move_data->vz_set_channel=yaw_error*3;
	if(num==2 || num==4)
	chassis_move_data->vz_set_channel=yaw_error*0.1f;
	if(num==13) 
	chassis_move_data->vz_set_channel=yaw_error*2;
	if(num==17)
	chassis_move_data->vz_set_channel=yaw_error*2;
	//获取速度
	chassis_move_data->absolute_chassis_speed.Vw = chassis_move_data->vz_set_channel;
	chassis_move_data->absolute_chassis_speed.Vx = chassis_move_data->vx_set_channel;
	chassis_move_data->absolute_chassis_speed.Vy = chassis_move_data->vy_set_channel;
	four_yaw_space(chassis_move_data);
	Robot_coordinate(&chassis_move_data->absolute_chassis_speed,final_yaw+90);
	//pid
	Auto_pid_caculate(chassis_move_data);
}

void num_0_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	
	else{
	//校准完成之后
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
//	position_judge(chassis_move_data);
//		if(position_judge_flag)
//		{
//			stop_3508(chassis_move_data);
//			stop_flag=1;
//		}
	if(chassis_move.GD_flag[0] && !time_save){time_save=1;last_time = HAL_GetTick();}
	current_time= HAL_GetTick();
	if(chassis_move.GD_flag[0])
	{
	if(current_time-last_time > 800){chassis_move_data->Paw_flag=Paw_2_catch;}
}
	if(chassis_move_data->Paw_flag==Paw_2_catch)
	{
	if(current_time-last_time > 1700){chassis_move_data->lift_circle_number=ceiling;chassis_move_data->step[0]=1;}
}
	if( chassis_move_data->step[0])
	{
		if(chassis_move_data->lift_over)
		{
				num++;
				clear_step(chassis_move_data);
				time_save=0;
				yaw_calibrate_flag=0;
				stop_flag=0;
		}
	}
}
}


void num_2_task(chassis_move_t *chassis_move_data)
{
	
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	yaw_calibrate_first(chassis_move_data);
	position_judge(chassis_move_data);
	if(position_judge_flag)
	{
		num++;
		position_judge_flag=0;
	}
}

void num_7_task(chassis_move_t *chassis_move_data)
{
	stop_3508(chassis_move_data);
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	
	else{
		num_7_flag = 1;
	put_first_task(chassis_move_data,3,4,0);
}
}

void num_5_task(chassis_move_t *chassis_move_data)
{	
	stop_3508(chassis_move_data);
	chassis_move_data->lift_circle_number=ground;
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	else{
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	current_time=HAL_GetTick();
			position_judge(chassis_move_data);
		if(position_judge_flag)
		{
			stop_3508(chassis_move_data);
			stop_flag=1;
		}
	if(chassis_move_data->GD_flag[0])
	{
		if(!time_save)
		{
		time_save=1;
		last_time=HAL_GetTick();
		}
	}
	if(chassis_move_data->GD_flag[0] && !chassis_move_data->step[0])
	{
		if(current_time-last_time>700){
			chassis_move_data->Paw_flag=Paw_2_catch;
			chassis_move_data->step[0]=1;
		}
	}
	if(chassis_move_data->step[0] && current_time-last_time>1100)
	{
			chassis_move_data->lift_circle_number=ceiling;
			clear_step(chassis_move_data);  //清除所有标志位
			yaw_calibrate_flag=0;
			time_save=0;
			stop_flag=0;
			position_judge_flag=0;

			num+=2;
	}
}
}

void num_10_task(chassis_move_t *chassis_move_data)
{
	num=10;
	chassis_move_data->lift_circle_number=ground;
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	
	else{
	//校准完成之后
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	position_judge(chassis_move_data);
		if(position_judge_flag)
		{
			stop_3508(chassis_move_data);
			stop_flag=1;
		}
	if(chassis_move.GD_flag[0] && !time_save){time_save=1;last_time = HAL_GetTick();}
	current_time= HAL_GetTick();
	if(chassis_move.GD_flag[0])
	{
	if(current_time-last_time > 800){chassis_move_data->Paw_flag=Paw_2_catch;}
}
	if(chassis_move_data->Paw_flag==Paw_2_catch)
	{
	if(current_time-last_time > 1300){chassis_move_data->step[0]=1;}
}
	if(chassis_move_data->step[0])
	{
		if(chassis_move_data->lift_over)
		{
				num++;
				clear_step(chassis_move_data);
				time_save=0;
				yaw_calibrate_flag=0;
				stop_flag=0;
		}
	}
}
}

void num_11_task(chassis_move_t *chassis_move_control_loop)
{
	chassis_move_control_loop->lift_circle_number=ground;
	Polar_To_Stright(chassis_move_control_loop,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag && num==11) //当校准标志为零时 进行校准并且仅校准一次 校准完成后再次开始
	yaw_calibrate(chassis_move_control_loop);
	else{
			position_judge(chassis_move_control_loop);
				if(position_judge_flag)
		{
			stop_3508(chassis_move_control_loop);
			stop_flag=1;
		}
		Auto_chassis_AGV_wheel_speed(chassis_move_control_loop);
		Auto_chassis_AGV_wheel_angle(chassis_move_control_loop);
		current_time=HAL_GetTick();
		if(chassis_move.GD_flag[1] && !time_save){time_save=1;last_time = HAL_GetTick();}
	current_time= HAL_GetTick();
	if(chassis_move.GD_flag[1])
	{
	if(current_time-last_time > 800){chassis_move_control_loop->Paw_flag=Paw_1_catch;chassis_move_control_loop->step[0]=1;}
}
		if(current_time-last_time > 1300&&chassis_move_control_loop->step[0]){
		chassis_move_control_loop->lift_circle_number=ceiling;
		num++; //完成
		clear_step(chassis_move_control_loop);  //清除所有标志位
		yaw_calibrate_flag=0;
		time_save=0;
		stop_flag=0;
	}
}
}

void turn_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	position_judge(chassis_move_data);
	if(position_judge_flag)
	{
		num++;
		if(num==17)
		chassis_move_data->lift_circle_number=ceiling;
		stop_3508(chassis_move_data);
	}
}

void disturb(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	static int return_back;
	chassis_move_data->absolute_chassis_speed.Vx = chassis_move_data->derta_X*0.25f;
	chassis_move_data->absolute_chassis_speed.Vy = chassis_move_data->derta_Y*0.25f;
	chassis_move_data->absolute_chassis_speed.Vw = 20;
	
	four_yaw_space(chassis_move_data);
	Robot_coordinate(&chassis_move_data->absolute_chassis_speed,final_yaw+90);		
	position_judge(chassis_move_data);
	if(position_judge_flag)
	{
		num+=return_back;
		if(num==24)return_back=-1;
		if(num==21)return_back=1;
	}
}

int turn_flag;
void turn_1_task(chassis_move_t *chassis_move_data)
{
	num=30;
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	position_judge(chassis_move_data);
	if(position_judge_flag)
	{	
		turn_flag=1;
		num=12;
		PID_CLEAR_ALL(chassis_move_data);
						chassis_move_data->my_chassis_mode = Chassis_Remote;
						osThreadSuspend(LED_FLOW_TASKHandle);												
						chassis_move_data->Auto_init_mode = 1;
						if(chassis_move_data->Remote_init_mode)
						remote_control_chassis_init(chassis_move_data);
						chassis_move_data->Remote_init_mode = 0;											
						chassis_remote_control_loop(chassis_move_data);			
						switch(chassis_move_data->chassis_RC->rc.s[0])
						{
							case 3:chassis_move_data->my_remote_mode = chassis_top_move;break;
							default :chassis_move_data->my_remote_mode =CHASSIS_ZERO_FORCE;
						}
	}
}

void solid_first_task(chassis_move_t *chassis_move_data,int Paw_catch,int Paw_loose,int GD)
{
	//校准完成之后
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	current_time= HAL_GetTick();
	position_judge(chassis_move_data);
	if(chassis_move_data->GD_flag[GD] && !chassis_move_data->step[0])
	{
		if(position_judge_flag)
		{
		slow_down=1;
		}
	if(!time_save)
		{
		time_save=1;
		last_time=HAL_GetTick();
		}if(current_time-last_time>1000){chassis_move_data->lift_circle_number=put_bamboo;chassis_move_data->step[0]=1;stop_3508(chassis_move_data);stop_flag=1;}
	}
	if(chassis_move_data->step[0])
	{
		stop_3508(chassis_move_data);
		if(chassis_move_data->lift_over)
		{
		if(current_time-last_time>1700)chassis_move_data->Paw_flag=Paw_loose;
		if(current_time-last_time>2300){chassis_move_data->lift_circle_number=second;chassis_move_data->step[1]=1;stop_3508(chassis_move_data);}
		if(chassis_move_data->lift_over && chassis_move_data->step[1])
		{
			if(current_time-last_time>3000){chassis_move_data->Paw_flag=Paw_catch;chassis_move_data->step[2]=1;stop_3508(chassis_move_data);}
		}
		if(current_time-last_time>3500 && chassis_move_data->step[2])
		{
			chassis_move_data->lift_circle_number=ceiling;
			if(chassis_move_data->lift_over)
			{
				chassis_move_data->step[3]=1;
				if(current_time-last_time>4000)
				{
				num++; //完成
				clear_step(chassis_move_data);  //清除所有标志位
				yaw_calibrate_flag=0;
				time_save=0;
				stop_flag=0;
				slow_down=0;
				}
			}
		}
	}
	}
}

//放置第二个竹子之后
void solid_second_task(chassis_move_t *chassis_move_data,int Paw_loose,int GD)
{
	//校准完成之后
	Auto_chassis_AGV_wheel_speed(chassis_move_data);
	Auto_chassis_AGV_wheel_angle(chassis_move_data);
	current_time= HAL_GetTick();
	position_judge(chassis_move_data);
	if(chassis_move_data->GD_flag[GD] && !chassis_move_data->step[0])
	{
		if(position_judge_flag)
		{
		slow_down=1;
		}
		if(!time_save)
		{
		time_save=1;
		last_time=HAL_GetTick();
		}
	if(current_time-last_time>1200){stop_3508(chassis_move_data);chassis_move_data->lift_circle_number=put_bamboo;chassis_move_data->step[0]=1;stop_flag=1;}
	}
	
	if(chassis_move_data->step[0])
	{
		stop_3508(chassis_move_data);
		if(chassis_move_data->lift_over)
		{
		if(current_time-last_time>2000)chassis_move_data->Paw_flag=Paw_loose; chassis_move_data->step[1]=1;stop_3508(chassis_move_data);
		}
	}
	
	if(current_time-last_time>2500 && chassis_move_data->step[1])
	{
		num++; //完成
		clear_step(chassis_move_data);  //清除所有标志位
		yaw_calibrate_flag=0;
		time_save=0;
		stop_flag=0;
		slow_down=0;
	}
}

void num_17_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	{yaw_calibrate(chassis_move_data); chassis_move_data->lift_circle_number=ceiling;}
	else{
		solid_first_task(chassis_move_data,1,2,1);
	}
}

void num_20_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	else
	{
		solid_second_task(chassis_move_data,2,1);
}
}


void num_3_task(chassis_move_t *chassis_move_data)
{
	Polar_To_Stright(chassis_move_data,Waypoint[num].Plo_Angle,Waypoint[num].Plo_Length);  //targetA  
	if(!yaw_calibrate_flag) //当校准标志为零时 进行校准并且仅校准一次
	yaw_calibrate(chassis_move_data); 
	else{
	solid_second_task(chassis_move_data,4,0);
}
}

int init_flag;
void cast_function(chassis_move_t *chassis_move_control_loop)
{
		switch(num)	
	{
		case 0:num_0_task(chassis_move_control_loop);break;  //取地上竹子
		case 1:num_1_task(chassis_move_control_loop,3,4,0);break;	 //放first
		case 2:num_2_task(chassis_move_control_loop);break;  //转点
		case 3:put_second_task(chassis_move_control_loop,4,0);break;  //放second

		case 10:num_10_task(chassis_move_control_loop);break;  //重置后取地上竹子
		case 11:num_11_task(chassis_move_control_loop);break; //取竹子
		case 12:turn_task(chassis_move_control_loop);break;  //转点
		case 13:put_first_task(chassis_move_control_loop,3,4,0);break; //放first
		case 14:turn_task(chassis_move_control_loop);break;  //转点
		case 15:num_3_task(chassis_move_control_loop);break;  //放second
		case 16:chassis_move_control_loop->lift_circle_number=put_bamboo;turn_task(chassis_move_control_loop);break;  //转点
		case 17:put_first_task(chassis_move_control_loop,1,2,1);break; //放first
		case 18:chassis_move_control_loop->lift_circle_number=ceiling;turn_task(chassis_move_control_loop);break; //转点
		case 19:turn_task(chassis_move_control_loop);break; //转点
		case 20:num_20_task(chassis_move_control_loop);break; //放second
	}
//	if(over)disturb(chassis_move_control_loop);
		Auto_pid_caculate(chassis_move_control_loop);  //巡逻
}





