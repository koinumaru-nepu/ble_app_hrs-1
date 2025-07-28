#ifndef __MAX30102_HAL_H
#define __MAX30102_HAL_H

#include "stdint.h" // ��������MCUϵ�и��ģ����� stm32f4xx_hal.h
// #include "i2c.h"

#include "nrf_delay.h"
/*==================================================================================================
   I2C ���� �� �豸��ַ
==================================================================================================*/


/*==================================================================================================
   MAX30102 �Ĵ�����ַ
==================================================================================================*/
// ״̬�Ĵ���
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03

// FIFO �Ĵ���
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07

// ���üĴ���
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A

// LED������ȼĴ���
#define REG_LED1_PA         0x0C // Red
#define REG_LED2_PA         0x0D // IR

// ����
#define REG_PILOT_PA        0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12

// �¶ȼĴ���
#define REG_TEMP_INTR       0x1F
#define REG_TEMP_FRAC       0x20
#define REG_TEMP_CONFIG     0x21

// ID �Ĵ���
#define REG_REV_ID          0xFE
#define REG_PART_ID         0xFF // �豸ID�Ĵ�����ֵΪ0x15

/*==================================================================================================
   ������������
==================================================================================================*/
void max30102_twi_master_init(void);
/**
 * @brief  ��ʼ��MAX30102������
 * @param  hi2c: ָ��I2C�����ָ��
 * @retval HAL_StatusTypeDef: HAL_OK ��ʾ�ɹ�, HAL_ERROR ��ʾʧ��
 */
// HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef *hi2c);
bool MAX30102_Init(void);

/**
 * @brief  ͨ��������λMAX30102
 * @retval HAL_StatusTypeDef: HAL����״̬
 */
// HAL_StatusTypeDef MAX30102_Reset(void);
bool MAX30102_Reset(void);
/**
 * @brief  ��MAX30102��FIFO�ж�ȡһ�����ݣ�Red LED �� IR LED��
 * @note   ÿ��ͨ����������18λ�ģ��洢��32λ�޷���������
 * @param  pun_red_led: ָ��洢Red LED���ݵı���ָ��
 * @param  pun_ir_led: ָ��洢IR LED���ݵı���ָ��
 * @retval HAL_StatusTypeDef: HAL����״̬
 */
// HAL_StatusTypeDef MAX30102_Read_FIFO(uint32_t *pun_red_led, uint32_t *pun_ir_led);
bool MAX30102_Read_FIFO(uint32_t *pun_red_led, uint32_t *pun_ir_led);
/**
 * @brief  ��MAX30102�ĵ����Ĵ���д��һ���ֽ�
 * @param  reg_addr: Ŀ��Ĵ�����ַ
 * @param  data: Ҫд�������
 * @retval HAL_StatusTypeDef: HAL����״̬
 */
// HAL_StatusTypeDef MAX30102_Write_Reg(uint8_t reg_addr, uint8_t data);
bool MAX30102_Write_Reg(uint8_t reg_addr, uint8_t data);
/**
 * @brief  ��MAX30102�ĵ����Ĵ�����ȡһ���ֽ�
 * @param  reg_addr: Ŀ��Ĵ�����ַ
 * @param  p_data: ָ��洢��ȡ���ݵı���ָ��
 * @retval HAL_StatusTypeDef: HAL����״̬
 */
// HAL_StatusTypeDef MAX30102_Read_Reg(uint8_t reg_addr, uint8_t *p_data);
bool MAX30102_Read_Reg(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes);


#endif /* __MAX30102_HAL_H */
