#ifndef __BSP_BMI088_H
#define __BSP_BMI088_H

#include "main.h"

extern SPI_HandleTypeDef *bmi088_spi;

void BMI_ACCEL_L(void);
void BMI_ACCEL_H(void);
void BMI088_GYRO_L(void);
void BMI088_GYRO_H(void);
uint8_t BMI088_read_write_byte(uint8_t tx_data);
void imu_pwm_set(uint16_t pwm);

#endif
