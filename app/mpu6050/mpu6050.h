#ifndef AT24C02_H__
#define AT24C02_H__
#include "nrf_delay.h"




#define MPU6050_GYRO_OUT        0x43
#define MPU6050_ACC_OUT         0x3B

#define ADDRESS_WHO_AM_I          (0x75U) // !< WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0x68U) // !<

//MPU6050�Ĵ���
#define MPU_SELF_TESTX_REG		0x0D	//�Լ�Ĵ���X
#define MPU_SELF_TESTY_REG		0x0E	//�Լ�Ĵ���Y
#define MPU_SELF_TESTZ_REG		0x0F	//�Լ�Ĵ���Z
#define MPU_SELF_TESTA_REG		0x10	//�Լ�Ĵ���A
#define MPU_SAMPLE_RATE_REG		0x19	//����Ƶ�ʷ�Ƶ��
#define MPU_CFG_REG				    0x1A	//���üĴ���
#define MPU_GYRO_CFG_REG		  0x1B	//���������üĴ���
#define MPU_ACCEL_CFG_REG		  0x1C	//���ٶȼ����üĴ���
#define MPU_MOTION_DET_REG		0x1F	//�˶���ֵⷧ���üĴ���
#define MPU_FIFO_EN_REG			  0x23	//FIFOʹ�ܼĴ���
#define MPU_I2CMST_CTRL_REG		0x24	//IIC�������ƼĴ���
#define MPU_I2CSLV0_ADDR_REG	0x25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU_I2CSLV0_REG			  0x26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU_I2CSLV0_CTRL_REG	0x27	//IIC�ӻ�0���ƼĴ���
#define MPU_I2CSLV1_ADDR_REG	0x28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU_I2CSLV1_REG			  0x29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU_I2CSLV1_CTRL_REG	0x2A	//IIC�ӻ�1���ƼĴ���
#define MPU_I2CSLV2_ADDR_REG	0x2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU_I2CSLV2_REG			  0x2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU_I2CSLV2_CTRL_REG	0x2D	//IIC�ӻ�2���ƼĴ���
#define MPU_I2CSLV3_ADDR_REG	0x2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU_I2CSLV3_REG			  0x2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU_I2CSLV3_CTRL_REG	0x30	//IIC�ӻ�3���ƼĴ���
#define MPU_I2CSLV4_ADDR_REG	0x31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU_I2CSLV4_REG			  0x32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU_I2CSLV4_DO_REG		0x33	//IIC�ӻ�4д���ݼĴ���
#define MPU_I2CSLV4_CTRL_REG	0x34	//IIC�ӻ�4���ƼĴ���
#define MPU_I2CSLV4_DI_REG		0x35	//IIC�ӻ�4�����ݼĴ���


#define MPU_PWR_MGMT1_REG		  0x6B	//��Դ�����Ĵ���1
#define MPU_PWR_MGMT2_REG		  0x6C	//��Դ�����Ĵ���2 

#define MPU_I2CMST_STA_REG		0x36	//IIC����״̬�Ĵ���
#define MPU_INTBP_CFG_REG		  0x37	//�ж�/��·���üĴ���
#define MPU_INT_EN_REG			  0x38	//�ж�ʹ�ܼĴ���
#define MPU_INT_STA_REG			  0x3A	//�ж�״̬�Ĵ���

#define MPU_I2CMST_DELAY_REG	0x67	//IIC������ʱ�����Ĵ���
#define MPU_SIGPATH_RST_REG		0x68	//�ź�ͨ����λ�Ĵ���
#define MPU_MDETECT_CTRL_REG	0x69	//�˶������ƼĴ���
#define MPU_USER_CTRL_REG		  0x6A	//�û����ƼĴ���
#define MPU_PWR_MGMT1_REG		  0x6B	//��Դ�����Ĵ���1
#define MPU_PWR_MGMT2_REG		  0x6C	//��Դ�����Ĵ���2 
#define MPU_FIFO_CNTH_REG		  0x72	//FIFO�����Ĵ����߰�λ
#define MPU_FIFO_CNTL_REG		  0x73	//FIFO�����Ĵ����Ͱ�λ
#define MPU_FIFO_RW_REG			  0x74	//FIFO��д�Ĵ���
#define MPU_DEVICE_ID_REG		  0x75	//����ID�Ĵ���

void mpu6050_twi_master_init(void);
bool mpu6050_init(void);

/**
  @brief Function for writing a MPU6050 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool mpu6050_register_write(uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading MPU6050 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool mpu6050_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying MPU6050 product ID.
  @retval true Product ID is what was expected
  @retval false Product ID was not what was expected
*/
bool mpu6050_verify_product_id(void);


bool MPU6050_ReadGyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z );
bool MPU6050_ReadAcc( int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z );

#endif


