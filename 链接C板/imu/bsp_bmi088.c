#include "main.h"
#include "bsp_bmi088.h"

SPI_HandleTypeDef *bmi088_spi;

void BMI_ACCEL_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void BMI_ACCEL_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t tx_data)
{
    uint8_t rx_data;
    
    HAL_SPI_TransmitReceive(bmi088_spi, &tx_data, &rx_data, 1, 1000);
    
    return rx_data;
}



void imu_pwm_set(uint16_t pwm)
{
    TIM10->CCR1 = (pwm);

}


