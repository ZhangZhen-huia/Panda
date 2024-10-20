/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_can.c
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-23-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "bsp_can.h"
#include "can.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "struct_typedef.h"
#include "InsTask.h"

extern CAN_HandleTypeDef hcan2;
uint8_t uwb_usart_data[9];

/**
  * @brief  Configures the CAN, transmit and receive by polling
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef can_filter_init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef  sFilterConfig;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
  if(hcan == &hcan2)
  {
    sFilterConfig.FilterBank = 14;
  }
  
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Start the CAN peripheral ###########################################*/
  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  return HAL_OK;
}


uint8_t                  TxData[UWB_TX_FIFO_SIZE];
uint8_t                  RxData[UWB_RX_FIFO_SIZE];
uint8_t                  uwb_rx_data_buf[UWB_RX_FIFO_SIZE];
uwb_info_t               uwb_data;
uwb_info_filter_t				 uwb_filter_data;
float x_window[WINDOW_SIZE] = {0};  // 用于存放最近的5次数据
float y_window[WINDOW_SIZE] = {0};  // 用于存放最近的5次数据
float yaw_window[WINDOW_SIZE] = {0};  // 用于存放最近的5次数据
int x_count = 0;  // 用于记录当前接收数据的次数
int y_count = 0;  // 用于记录当前接收数据的次数
int yaw_count=0;

// 实时移动平均滤波函数
float moving_average_filter(float new_data, float *window, int *count) {
    float sum = 0;

    // 更新窗口中的数据
    window[*count % WINDOW_SIZE] = new_data;

    // 更新计数器
    (*count)++;

    // 判断是否已经有足够的数据进行滤波（至少窗口大小的数据）
    int valid_count = (*count < WINDOW_SIZE) ? *count : WINDOW_SIZE;  // 如果少于窗口大小，计算当前已有的数据个数

    // 计算窗口内的平均值
    for (int i = 0; i < valid_count; i++) {
        sum += window[i];
    }
    return sum / valid_count;  // 返回滤波后的数据
}

uint32_t can_rx_flag = 0;
extern fp32 INS_angle[3];      //euler angle, unit rad.欧拉角 单位 rad
uint16_t yaw;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)//  CAN FIFO0的中断回调函数，在里面完成数据的接收
{
	CAN_RxHeaderTypeDef RxHeader;
	if(hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxHeader,RxData);
		if (RxHeader.StdId == UWB_CAN_RX_ID)
		{
			uint8_t index = 0;
			static uint8_t *p = (uint8_t *)&uwb_rx_data_buf;
			uint8_t unpackstep = 0;
			
			if (RxHeader.DLC == 8)
			{
				for (index = 0; index < 8; index++)
				{
					*p++ = RxData[index];
				}
				unpackstep++;
			}
			else if(RxHeader.DLC == 6)
			{
				for (index = 0; index <6; index++)
				{
					*p++ = RxData[index];
				}
				p = (uint8_t *)&uwb_rx_data_buf;
				memcpy(&uwb_data, &uwb_rx_data_buf, sizeof(uwb_info_t));
				//yaw = (INS_angle[0]+3.14f)*100/3.14f*180;
				yaw = ImuRobot.Yaw+180.0f;
				uwb_filter_data.uwb_filtered_x=moving_average_filter(uwb_data.coor_x,x_window,&x_count);
				uwb_filter_data.uwb_filtered_y=moving_average_filter(uwb_data.corr_y,y_window,&y_count);
//				uwb_filter_data.uwb_filtered_yaw=moving_average_filter(uwb_data.yaw,yaw_window,&yaw_count);
				uwb_usart_data[0]=0x55;
				uwb_usart_data[1]=(uint8_t)(uwb_filter_data.uwb_filtered_x/256);
				uwb_usart_data[2]=(uint8_t)(uwb_filter_data.uwb_filtered_x%256);
				uwb_usart_data[3]=(uint8_t)(uwb_filter_data.uwb_filtered_y/256);
				uwb_usart_data[4]=(uint8_t)(uwb_filter_data.uwb_filtered_y%256);
				uwb_usart_data[5]=(uint8_t)(yaw/256);
				uwb_usart_data[6]=(uint8_t)(yaw%256);
				uwb_usart_data[7]=0x55;

//				uwb_usart_data[6]=(uint8_t)(uwb_filter_data.uwb_filtered_yaw/256);
//				uwb_usart_data[7]=(uint8_t)(uwb_filter_data.uwb_filtered_yaw%256);
				while(HAL_UART_Transmit(&huart6,uwb_usart_data,8,HAL_MAX_DELAY)!=HAL_OK);
		
				can_rx_flag ++;
			}
		}
	}
}




uint8_t paw_data;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(HAL_UART_Receive_IT(&huart1,&paw_data,1)==HAL_OK)
		{
				switch(paw_data)
				{
					case 1:
						HAL_GPIO_WritePin(paw_1_GPIO_Port,paw_1_Pin,GPIO_PIN_SET);break;
					case 2:
						HAL_GPIO_WritePin(paw_1_GPIO_Port,paw_1_Pin,GPIO_PIN_RESET);break;
				
					case 3:
						HAL_GPIO_WritePin(paw_2_GPIO_Port,paw_2_Pin,GPIO_PIN_SET);break;
					case 4:
						HAL_GPIO_WritePin(paw_2_GPIO_Port,paw_2_Pin,GPIO_PIN_RESET);break;
					case 5:
						HAL_GPIO_WritePin(paw_1_GPIO_Port,paw_1_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(paw_2_GPIO_Port,paw_2_Pin,GPIO_PIN_RESET);
					break;
				}

		}
}


//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//		while(HAL_UART_Transmit_IT(&huart1,uwb_usart_data,6)!=HAL_OK);

//}


