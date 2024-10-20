#include "chassis_task.h"
#include "detect_task.h"
#include "math.h"
#include "ops.h"
#include "filter.h"
#include "PAW_TASK.h"
#include "remote_task.h"
#include "freertos.h"
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
//车逆时针对应爪子
//#define PAW1_Pin GPIO_PIN_12
//#define PAW1_GPIO_Port GPIOH
//#define PAW2_Pin GPIO_PIN_11
//#define PAW2_GPIO_Port GPIOH

#define PAW_CATCH HAL_GPIO_WritePin(paw->PAW_GPIOx,paw->PAW_Pin,GPIO_PIN_SET) //抓
#define PAW_LOOSE HAL_GPIO_WritePin(paw->PAW_GPIOx,paw->PAW_Pin,GPIO_PIN_RESET) //松

paw_type paw_1=
{
	.PAW_GPIOx=PAW1_GPIO_Port,
	.PAW_Pin=PAW1_Pin
};

paw_type paw_2=
{
	.PAW_GPIOx=PAW2_GPIO_Port,
	.PAW_Pin=PAW2_Pin
};


//爪子初始化
void Paw_Init(paw_type*paw);
//爪子状态获取
void Paw_Get(paw_type*paw);

void PAW_Task(void const * argument)
{
	Paw_Init(&paw_1);
	Paw_Init(&paw_2);

	while(1)
	{
//		Paw_Get(&paw_1);
//		Paw_Get(&paw_2);
		
	}
}

void Paw_Get(paw_type*paw)
{
	if(chassis_move.chassis_RC->rc.s[1]==3)
	{
		if(chassis_move.chassis_RC->rc.ch[0]>10)
		{
			PAW_CATCH;
		}
		else
		{
			PAW_LOOSE;
		}
	}	
	if(chassis_move.derta_X <13 && chassis_move.derta_Y <13)
	{
			PAW_CATCH;
	}
	
}

void Paw_Init(paw_type*paw)
{
	//爪子初始化均为松开状态
	PAW_CATCH;
	osDelay(500);
	PAW_LOOSE;
	osDelay(500);
	PAW_CATCH;

}










