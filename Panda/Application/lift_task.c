#include "lift_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "chassis_task.h"

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
const Revice_Motro* Lift_motor_data; //定义电机数据包

pid_type_def Lift_speed_pid;//定义pid结构体
pid_type_def Lift_ecd_pid;


int16_t  last_ecd;
int64_t  ecd_sum=0;

/*13放竹子，0夹竹子，35最高点，运竹子*/
int8_t circle_number=0;
int8_t temp;
void height_get(void);



void lift_task(void const * argument)
{
	lift_motor_init();
	while(1)
	{
		/*过零检测*/
		height_get();//获取值
		static int16_t ecd_error;
		ecd_error = Lift_motor_data->ecd -last_ecd;
		ecd_error = ecd_error >  4095 ?   ecd_error - 8191 : ecd_error;
		ecd_error = ecd_error < -4095 ?   ecd_error + 8191 : ecd_error;
		ecd_sum -= ecd_error;
		last_ecd = Lift_motor_data->ecd;
		
		if(fabs(-ecd_sum-chassis_move.lift_circle_number*8191)<8000)
		{
			chassis_move.lift_over=1;
		}
		/*circle_number最大为40（抬升电机在最下面的时候）*/
		PID_calc(&Lift_ecd_pid,-ecd_sum,chassis_move.lift_circle_number*8191);//位置PID计算
		PID_calc(&Lift_speed_pid,Lift_motor_data->rpm,Lift_ecd_pid.out);//速度PID计算
		/*对抬升电机赋值*/
		
				if(!toe_is_error(DBUS_TOE))
		{
		CAN_cmd_Lift(Lift_speed_pid.out,0,0,0);
			
		}
		else
		{
		PID_clear(&Lift_speed_pid);
		PID_clear(&Lift_ecd_pid);
		CAN_cmd_Lift(0,0,0,0);
		chassis_move.lift_circle_number=0;
		}
		osDelay(1);
	}
}

void block_test(void)
{
	
	
}

void height_get(void)
{
	if(chassis_move.chassis_RC->rc.s[1]==3)
	{
		if(chassis_move.chassis_RC->rc.s[0]==3)
		{			
			circle_number=chassis_move.chassis_RC->rc.ch[3]/20;
			if(circle_number!=0)
			{
			temp=circle_number;
			}
			else
			{
			circle_number=temp;
			}
		}
	}
}

void lift_motor_init(void)
{
	static float PID_SPEED[3]={LIFT_MOTOR_SPEED_PID_KP,LIFT_MOTOR_SPEED_PID_KI,LIFT_MOTOR_SPEED_PID_KD};//PID参数设置
	static float PID_ANGLE[3]={LIFT_MOTOR_ANGLE_PID_KP,LIFT_MOTOR_ANGLE_PID_KI,LIFT_MOTOR_ANGLE_PID_KD};//PID参数设置
	
	PID_init(&Lift_speed_pid,PID_POSITION,PID_SPEED,LIFT_MOTOR_SPEED_PID_MAX_OUT,LIFT_MOTOR_SPEED_PID_MAX_IOUT);//初始化PID
	PID_init(&Lift_ecd_pid,PID_POSITION,PID_ANGLE,LIFT_MOTOR_ANGLE_PID_MAX_OUT,LIFT_MOTOR_ANGLE_PID_MAX_IOUT);//初始化PID
    Lift_motor_data = get_Lift_Motor_Point();//获取电机的数据，接收电机的反馈结构体
}














