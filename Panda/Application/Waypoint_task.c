#include "chassis_task.h"
#include "detect_task.h"
#include "Waypoint_task.h"
#include "Can_recive.h"
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

Waypoints_t Waypoint[50];
void Sensor_Get(void);

void Waypoint_task(void const * argument)
{
	Waypoint_Init();
	chassis_move.Paw_flag = 5;
	while(HAL_UART_Transmit(&huart7,&chassis_move.Paw_flag,1,100)!=HAL_OK);

	HAL_UART_Transmit_IT(&huart7,&chassis_move.Paw_flag,1);
		while(1)
	{
		Sensor_Get();
		
		//CAN1_Send_Test(0x666);
	}
}

void Waypoint_Init(void)
{
	Waypoint_Write(0,-139.0f ,150.0f); //取竹子
	Waypoint_Write(1,-133.0f, 260.0f);  //自转，放first
	Waypoint_Write(2,-100.0f, 210.0f);  //不自转，转点
	Waypoint_Write(3,-106.0f , 270.0f); //自转，放second
	
	/*回弹*/
	Waypoint_Write(4,-100.0f , 200.0f); //不自转，后退

	//重试之后的夹取的竹子
	
	Waypoint_Write(10,-139.0f , 170.0f);//取竹子

	Waypoint_Write(11,-70.0f , 190.0f);//取竹子
	
	Waypoint_Write(12,-95.0f , 160.0f); // 转点

	Waypoint_Write(13,-85.0f , 35.0f);//放first

	Waypoint_Write(14,-145.0f , 140.0f);// 转点

	Waypoint_Write(15,-137.0f , 35.0f);// 放second

	Waypoint_Write(16,175.0f , 150.0f);//转点

	Waypoint_Write(17,175.0f , 20.0f);// 放first

	Waypoint_Write(18,145.0f , 100.0f);//转点

	Waypoint_Write(19,100.0f , 140.0f);//转点
	
	Waypoint_Write(20,75.0f , 20.0f);// 放second
/*  巡逻  */
	Waypoint_Write(21,107.0f , 110.0f); //
	
	Waypoint_Write(22,160.0f , 120.0f); //

	Waypoint_Write(23,-140.0f , 110.0f); //
	
	Waypoint_Write(24,-80.0f , 110.0f); //
	
	Waypoint_Write(30,-105.0f , 160.0f); //
///////////////////////////////////////////////////////////////
//		Waypoint_Write(10,-140.0f , 150.0f);//取竹子

//	Waypoint_Write(11,-70.0f , 208.0f);//取竹子
//	
//	Waypoint_Write(12,-90.0f , 140.0f); // 转点

//	Waypoint_Write(13,-85.0f , 35.0f);//放first

//	Waypoint_Write(14,-145.0f , 140.0f);// 转点

//	Waypoint_Write(15,-137.0f , 35.0f);// 放second

//	Waypoint_Write(16,175.0f , 140.0f);//转点

//	Waypoint_Write(17,167.0f , 30.0f);// 放first

//	Waypoint_Write(18,145.0f , 100.0f);//转点

//	Waypoint_Write(19,100.0f , 140.0f);//转点
//	
//	Waypoint_Write(20,65.0f , 20.0f);// 放second
//	Waypoint_Write(19,103, 118.0f);// 档它

	
	
//	/*最左边的竹子对应的蓝色区域的竹子 逆时针*/
//	Waypoint_Write(6,-135.0f , 95.0f);
//	Waypoint_Write(7,-120.0f , 95.0f);
//	Waypoint_Write(8,-105.0f , 95.0f);
//	Waypoint_Write(9,-95.0f , 95.0f);
//	Waypoint_Write(10,-75.0f  , 95.0f);
//	Waypoint_Write(11,-60.0f  , 95.0f);
//	Waypoint_Write(12,-45.0f  , 95.0f);
//	Waypoint_Write(13,-30.0f  , 95.0f);
//	Waypoint_Write(14,-15.0f  , 95.0f);
//	Waypoint_Write(15,0.0f  , 95.0f);
//	Waypoint_Write(16, 15.0f  , 95.0f);
//	Waypoint_Write(17, 30.0f  , 95.0f);
//	Waypoint_Write(18, 45.0f  , 95.0f);
//	Waypoint_Write(19, 60.0f  , 95.0f);
//	Waypoint_Write(20, 75.0f  , 95.0f);
//	Waypoint_Write(21, 90.0f  , 95.0f);
//	Waypoint_Write(22,105.0f , 95.0f);
//	Waypoint_Write(23,120.0f , 95.0f);
//	Waypoint_Write(24,135.0f , 95.0f);
//	Waypoint_Write(25,150.0f , 95.0f);
//	Waypoint_Write(26,165.0f , 95.0f);
//	Waypoint_Write(27,180.0f , 95.0f);
//	Waypoint_Write(28,-170.0f , 95.0f);
//	Waypoint_Write(29,-150.0f , 95.0f);
}

void Waypoint_Write(uint8_t number,float Plo_Angle,float Plo_Length)
{
	Waypoint[number].Plo_Angle		= Plo_Angle;
	Waypoint[number].Plo_Length 	= Plo_Length;
}


inline uint8_t GD_key_3_get(void)
{
	if(HAL_GPIO_ReadPin(GD3_GPIO_Port,GD3_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t GD_key_2_get(void)
{
	if(HAL_GPIO_ReadPin(GD2_GPIO_Port,GD2_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t GD_key_1_get(void)
{
	if(HAL_GPIO_ReadPin(GD1_GPIO_Port,GD1_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t touch_key_1_get(void)
{
	if(HAL_GPIO_ReadPin(TOUCH1_GPIO_Port,TOUCH1_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t touch_key_2_get(void)
{
	if(HAL_GPIO_ReadPin(TOUCH2_GPIO_Port,TOUCH2_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}
inline uint8_t touch_key_3_get(void)
{
	if(HAL_GPIO_ReadPin(TOUCH3_GPIO_Port,TOUCH3_Pin) == GPIO_PIN_RESET)
		return 1;
	else 
		return 0;
}

void Sensor_Get(void)
{
	chassis_move.GD_flag[0]=GD_key_1_get();
	chassis_move.GD_flag[1]=GD_key_2_get();
	chassis_move.GD_flag[2]=GD_key_3_get();
	chassis_move.touch_flag[0]=touch_key_1_get();
	chassis_move.touch_flag[1]=touch_key_2_get();
	chassis_move.touch_flag[2]=touch_key_3_get();
}
