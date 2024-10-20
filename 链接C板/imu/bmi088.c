#include "main.h"
#include "bsp_bmi088.h"
#include "bmi088.h"
#include "BMI088reg.h"
#include "MyDWT.h"
#include "math.h"


float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

float gyroDiff[3], gNormDiff;

uint8_t caliOffset = 0;
int16_t caliCount = 0;

IMU_DATA_T BMI088;

static uint8_t error = BMI088_NO_ERROR;
static uint8_t res = 0;
static uint8_t write_reg_num = 0;

uint8_t BMI088_init_first(SPI_HandleTypeDef *BMI088_SPI, uint8_t CALI);
void Calibrate_MPU_OFFSET(IMU_DATA_T *bmi088);

float a;

//�洢BMI088���ٶȴ������ļĴ���д�����ݣ��鿴оƬ�ֲ����֪����������
//���Ƽ��ٶȴ������ĵ�Դ�����ü��ٶȴ������ĵ�Դģʽ������ģʽ���������Ƶ�ʣ����ٶȴ����������̣������ж�1��GPIO���ƣ�ӳ������׼���жϵ��ж�1
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
					/*		�Ĵ�����ַ					д��Ĵ�����ֵ							�����жϣ�����Ƿ�д��ɹ�			*/
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};


//�洢BMI088�����Ǵ������Ĵ���д�����ݵ�����
//���̣������͹��ģ����������ǵ�����������ʺ�ʹ�ܣ������ж�3���ж�4��GPIO���ƣ�ӳ������׼�������жϵ��ж�3
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
				/*		�Ĵ�����ַ			д��Ĵ�����ֵ						����Ƿ�д��ɹ� 	*/
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},	
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);


//�궨�壬д���ٶȼƼĴ���һ��
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI_ACCEL_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI_ACCEL_H();                     \
    }
//�궨�壬�����ٶȼƼĴ���һ��
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI_ACCEL_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI_ACCEL_H();                    \
    }
		
//�궨�壬�����ٶȼƶ���Ĵ�����������
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI_ACCEL_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI_ACCEL_H();                       \
    }

//�궨�壬д�����ǵ����Ĵ���
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_H();                     \
    }
//�궨�壬�������ǵ����Ĵ���
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_H();                     \
    }
//�궨�壬�������Ƕ���Ĵ���
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_H();                         \
    }



void BMI088_INIT(SPI_HandleTypeDef *BMI088_SPI, uint8_t calibrate)
{
    while (BMI088_init_first(BMI088_SPI, calibrate));
    caliOffset = 1;
}

//��ʼ��BMI088
uint8_t BMI088_init_first(SPI_HandleTypeDef *BMI088_SPI, uint8_t CALI)
{
    bmi088_spi = BMI088_SPI;//�����ֵ
    error = BMI088_NO_ERROR;//��ʼ����ERR

		/*��ʼ�����ٶȼƺ�������*/
    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    if(CALI)
        Calibrate_MPU_OFFSET(&BMI088);//У׼MPU��ƫ����
    else
        {	
						/*ʹ��Ԥ���ƫ����GxOFFSET��GyOFFSET��GzOFFSET*/
            BMI088.GYRO_OFFSET[0] = GxOFFSET;
            BMI088.GYRO_OFFSET[1] = GyOFFSET;
            BMI088.GYRO_OFFSET[2] = GzOFFSET;
						
						/*���ü��ٶȼ�����*/
            BMI088.Gyro_normal = gNORM;
            BMI088.ACCELSCALE = 9.81f / BMI088.Gyro_normal;
					
						/*�¶�У׼����TempWhenCaliΪ40��*/
            BMI088.TempWhenCali = 40;
        }

    // imu_offset_accquier(&BMI088);

    return error;

}


uint8_t bmi088_accel_init(void)
{
    //��ID
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
		
		//�ٴζ�ID
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // �����λ�������ø�λֵ��Ϊ��ȷ�����ٶȼƴ�����֪�ĳ�ʼ״̬
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    //��ID��ȷ�����ú�ͨ����Ȼ����
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
				//д��Ĵ���
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);
				
				//��ȡ�Ĵ���
        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);
				
				//����Ƿ���д���ֵƥ�䡣�����ƥ�䣬����Ӧ�Ĵ��������ӵ�error������
        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    // check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);
	
    // check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        HAL_Delay(1);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            write_reg_num--;
            // return write_BMI088_gyro_reg_data_error[write_reg_num][2];
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

void Calibrate_MPU_OFFSET(IMU_DATA_T *bmi088)
{
		//��ʼʱ��Ͳɼ������Լ���������ȡ�����ݵĻ�����
    static float starttime;
    static uint16_t Calitimes = 21000;
    uint8_t buf[8] = {0};

    int16_t bmi088_raw_temp;
		
		//�����Сֵ
    float gyro_max[3], gyro_min[3];
		
		//���ڼ������������ݵļ���
    float gyro_normal_temp = 0, gyro_normal_min = 0, gyro_normal_max = 0;

    starttime = DWT_GetTimeline_s();//��λ����

    do
    {
				//���У׼ʱ�䳬��10�룬��ʹ��Ԥ���ƫ����������ϵ�����˳�У׼
        if((DWT_GetTimeline_s() - starttime) > 10.0f)
        {
            bmi088->GYRO_OFFSET[0] = GxOFFSET;
            bmi088->GYRO_OFFSET[1] = GyOFFSET;
            bmi088->GYRO_OFFSET[2] = GzOFFSET;
            bmi088->Gyro_normal = gNORM;//����
            bmi088->TempWhenCali = 40;
            break;
        }

        DWT_Delay(0.005);
        bmi088->Gyro_normal = 0;
        bmi088->GYRO_OFFSET[0] = 0;
        bmi088->GYRO_OFFSET[1] = 0;
        bmi088->GYRO_OFFSET[2] = 0;

        for (uint16_t i = 0; i < Calitimes; i++)
        {		
						//��ȡ���ٶȼ�X������ĵ��ֽڣ�BMI088_ACCEL_XOUT_L���ٶȼ�X������ĵ��ֽڼĴ�����ַ
						//�ú�����������������������BMI088_ACCEL_XOUT_L�����6���Ĵ�����ֵ���պð�xyz�������
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->ACCEL[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->ACCEL[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->ACCEL[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
					
						//ʹ��ŷ����÷�����ʽ������ٶ�������ģ��
            gyro_normal_temp = sqrtf(bmi088->ACCEL[0] * bmi088->ACCEL[0] +
                              bmi088->ACCEL[1] * bmi088->ACCEL[1] +
                              bmi088->ACCEL[2] * bmi088->ACCEL[2]);
						//�ۼӼ��ٶ�������ģ��
            bmi088->Gyro_normal += gyro_normal_temp;
						
						//��ID���ж�ID�Ƿ�������Ȼ��ϳ�xyz�������ݣ����ۼӵ���Ӧ��ƫ�������������ں�����У׼����
            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->GYRO[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GYRO_OFFSET[0] += bmi088->GYRO[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->GYRO[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GYRO_OFFSET[1] += bmi088->GYRO[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->GYRO[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GYRO_OFFSET[2] += bmi088->GYRO[2];
            }

            // ��¼���ݼ���
            if (i == 0)
            {
                gyro_normal_max = gyro_normal_temp;
                gyro_normal_min = gyro_normal_temp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyro_max[j] = bmi088->GYRO[j];
                    gyro_min[j] = bmi088->GYRO[j];
                }
            }
            else
            {
                if (gyro_normal_temp > gyro_normal_max)
                    gyro_normal_max = gyro_normal_temp;
                if (gyro_normal_temp < gyro_normal_min)
                    gyro_normal_min = gyro_normal_temp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (bmi088->GYRO[j] > gyro_max[j])
                        gyro_max[j] = bmi088->GYRO[j];
                    if (bmi088->GYRO[j] < gyro_min[j])
                        gyro_min[j] = bmi088->GYRO[j];
                }
            }

            // ���ݲ��������Ϊ�յ������ţ�������У׼
            gNormDiff = gyro_normal_max - gyro_normal_min;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyro_max[j] - gyro_min[j];
            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f ||
                gyroDiff[1] > 0.15f ||
                gyroDiff[2] > 0.15f)
                break;
            DWT_Delay(0.0005);
        }

        // ȡƽ��ֵ�õ��궨���
        bmi088->Gyro_normal /= (float)Calitimes;
        for (uint8_t i = 0; i < 3; i++)
            bmi088->GYRO_OFFSET[i] /= (float)Calitimes;

        // ��¼�궨ʱIMU�¶�
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088->TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++;  
    } while (gNormDiff > 0.5f ||
             fabsf(bmi088->Gyro_normal - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f ||
             gyroDiff[1] > 0.15f ||
             gyroDiff[2] > 0.15f ||
             fabsf(bmi088->GYRO_OFFSET[0]) > 0.01f ||
             fabsf(bmi088->GYRO_OFFSET[1]) > 0.01f ||
             fabsf(bmi088->GYRO_OFFSET[2]) > 0.01f);
						//�����������ƫ����������ֵ�������У׼��
		
    bmi088->ACCELSCALE = 9.81f / bmi088->Gyro_normal; 

}

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}

uint8_t buf1[8];
void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
		

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
		for(uint8_t i=0;i<8;i++)
		{
			buf1[i]=buf[i];
		}
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = -bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = -bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = -bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = -bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}


//void BMI088_Read(IMU_DATA_T *bmi088)
//{
//    static uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
//    static int16_t bmi088_raw_temp;

//    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

//    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
//    bmi088->ACCEL[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->ACCELSCALE;
//    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
//    bmi088->ACCEL[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->ACCELSCALE;
//    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
//    bmi088->ACCEL[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->ACCELSCALE;

//    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
//    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
//    {
//        if (caliOffset)
//        {
//            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
//            bmi088->GYRO[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GYRO_OFFSET[0];
//            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
//            bmi088->GYRO[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GYRO_OFFSET[1];
//            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
//            bmi088->GYRO[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GYRO_OFFSET[2];
//        }
//        else
//        {
//            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
//            bmi088->GYRO[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
//            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
//            bmi088->GYRO[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
//            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
//            bmi088->GYRO[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
//        }
//    }
//    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

//    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

//    if (bmi088_raw_temp > 1023)
//    {
//        bmi088_raw_temp -= 2048;
//    }

//    bmi088->Temp = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
//}











