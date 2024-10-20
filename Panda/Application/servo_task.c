#include "led_flow_task.h"
#include "tim.h"
#include "cmsis_os.h"
#include "main.h"
#include "gpio.h"


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
　　　┃ 　　　　　　　┓
　　　┃ 　　　　　　　┏┛
　　　┗┓┓┏━┳┓┏┛ + + + +
　　　　┃┫┫　┃┫┫
　　　　┗┻┛　┗┻┛+ + + +

'''*/
#define servo_set_angle(tim,channel,x)			__HAL_TIM_SetCompare(&tim,channel,x/180*2000+500)

/**
	*PWM信号要求
	*		周期�?20ms
	*		频率�?50hz
	*		高电平时间为0.5-2.5ms
	*		又CRR=500时，高电平时�?500/20000*20=0.5ms
	*		CRR=2500时，高电平时间为2500/20000*20=2.5ms
	*		故CRR的值为500-2500
	*/

void servo_task(void const * argument)
{
	/*这两�?�? HAL_TIM_PWM_Start 已经开过了，感觉不用再加了*/

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	while(1)
	{
		servo_set_angle(htim2, TIM_CHANNEL_1, 0.0);
		servo_set_angle(htim2, TIM_CHANNEL_2, 30.0);
    servo_set_angle(htim2, TIM_CHANNEL_3, 60.0);
		servo_set_angle(htim2, TIM_CHANNEL_4, 90.0);
    servo_set_angle(htim8, TIM_CHANNEL_1, 120.0);
    servo_set_angle(htim8, TIM_CHANNEL_2, 150.0);
    servo_set_angle(htim8, TIM_CHANNEL_3, 0.0);
		servo_set_angle(htim8, TIM_CHANNEL_4, 0.0);
    osDelay(1000);
		
	}
}





