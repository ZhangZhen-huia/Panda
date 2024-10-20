#ifndef _PAW_TASK_H
#define _PAW_TASK_H

typedef struct
{
	GPIO_TypeDef* PAW_GPIOx;
	uint16_t PAW_Pin;
	uint8_t ready;
}paw_type;



#define Paw_Delay osDelay(100)
void PAW_Task(void const * argument);
void Paw_Get(paw_type*paw);

#endif
