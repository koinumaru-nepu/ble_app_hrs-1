

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "max30102.h"
#include "sdk_config.h"


#if max30102_en == 1

/*==================================================================================================
   I2C ���� �� �豸��ַ
==================================================================================================*/
// #define TWI_TEST 0

// #if TWI_TEST == 1 //TEST
#if test_on_board == 1 //TEST

#define TWI_SCL_M           22         //I2C SCL????
#define TWI_SDA_M           23   
      //I2C SDA????
      
#else               //REAL
#define TWI_SCL_M           10         //I2C SCL????
#define TWI_SDA_M           9         //I2C SDA????

#endif
// MAX30102 ��7λI2C��ַ�� 0x57��HAL�⺯�����Զ�������дλ�����������ṩ����һλ��ĵ�ַ��
// #define MAX30102_I2C_ADDR   (0xAE)
#define MPU6050_ADDRESS_LEN  1         //MPU6050��ַ����
#define MPU6050_ADDRESS     (0xAE>>1)  //MPU6050��ַ
#define MPU6050_WHO_AM_I     0x68U     //MPU6050 ID
//TWI驱动程序实例ID,ID和外设编号对应，0:TWI0  1:TWI1
#define TWI_INSTANCE_ID     0
//TWI传输完成标志
static volatile bool m_xfer_done = false;
//定义TWI驱动程序实例，名称为m_twi
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//TWI事件处理函数
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //判断TWI事件类型
	  switch (p_event->type)
    {
        //传输完成事件
			  case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;//置位传输完成标志
            break;
        default:
            break;
    }
}
//TWI初始化
void max30102_twi_master_init(void)
{
    ret_code_t err_code;
    //定义并初始化TWI配置结构体
    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_M,  //定义TWI SCL引脚
       .sda                = TWI_SDA_M,  //定义TWI SDA引脚
       .frequency          = NRF_DRV_TWI_FREQ_100K, //TWI速率
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWI优先级
       .clear_bus_init     = false//初始化期间不发送9个SCL时钟
    };
    //初始化TWI
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
	//检查返回的错误代码
    APP_ERROR_CHECK(err_code);
    //使能TWI
    nrf_drv_twi_enable(&m_twi);
}


// 定义一个静态的I2C句柄指针，由Init函数初始化
// static I2C_HandleTypeDef *max30102_i2c_handle;

/**
 * @brief  向MAX30102的单个寄存器写入一个字节
 * @param  reg_addr: 目标寄存器地址
 * @param  data: 要写入的数据
 * @retval HAL_StatusTypeDef: HAL操作状态
 */
// HAL_StatusTypeDef MAX30102_Write_Reg(uint8_t reg_addr, uint8_t data)
// {
//     // 使用HAL库的内存写函数，它会自动处理Start、设备地址、寄存器地址、数据和Stop信号
//     return HAL_I2C_Mem_Write(max30102_i2c_handle, MAX30102_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
// }

bool MAX30102_Write_Reg(uint8_t register_address, uint8_t value)
{
	  ret_code_t err_code;
	  uint8_t tx_buf[MPU6050_ADDRESS_LEN+1];
	
	  //准备写入的数据
		tx_buf[0] = register_address;
        tx_buf[1] = value;
		//TWI传输完成标志设置为false
		m_xfer_done = false;
		//写入数据
    err_code = nrf_drv_twi_tx(&m_twi, MPU6050_ADDRESS, tx_buf, MPU6050_ADDRESS_LEN+1, false);
	  //等待TWI总线传输完成
    while (m_xfer_done == false);
	  if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		return true;	
}

/**
 * @brief  从MAX30102的单个寄存器读取一个字节
 * @param  reg_addr: 目标寄存器地址
 * @param  p_data: 指向存储读取数据的变量指针
 * @retval HAL_StatusTypeDef: HAL操作状态
 */
// HAL_StatusTypeDef MAX30102_Read_Reg(uint8_t reg_addr, uint8_t *p_data)
// {
//     // 使用HAL库的内存读函数
//     return HAL_I2C_Mem_Read(max30102_i2c_handle, MAX30102_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, 1, HAL_MAX_DELAY);
// }
bool MAX30102_Read_Reg(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
	  ret_code_t err_code;
	
	  //TWI传输完成标志设置为false
		m_xfer_done = false;
	  err_code = nrf_drv_twi_tx(&m_twi, MPU6050_ADDRESS, &register_address, 1, true);
	  //等待TWI总线传输完成
    while (m_xfer_done == false);
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
	  //TWI传输完成标志设置为false
		m_xfer_done = false;
	  err_code = nrf_drv_twi_rx(&m_twi, MPU6050_ADDRESS, destination, number_of_bytes);
		//等待TWI总线传输完成
    while (m_xfer_done == false);
		if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		return true;
}
/**
 * @brief  通过软件复位MAX30102
 * @retval HAL_StatusTypeDef: HAL操作状态
 */
bool MAX30102_Reset(void)
{
    // 向模式配置寄存器写入复位位
    if (MAX30102_Write_Reg(REG_MODE_CONFIG, 0x40) != true)
    {
        return false;
    }
    // 等待复位完成，根据数据手册，复位后所有寄存器恢复默认值，标志位被清除
    nrf_delay_ms(10); // 延时一小段时间确保复位完成
    return true;
}

/**
 * @brief  初始化MAX30102传感器
 * @param  hi2c: 指向I2C句柄的指针
 * @retval HAL_StatusTypeDef: HAL_OK 表示成功, HAL_ERROR 表示失败
 */
// HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef *hi2c)
// {
//     uint8_t part_id = 0;

//     // 1. 保存I2C句柄
//     max30102_i2c_handle = hi2c;

//     // 2. 软复位传感器
//     if (MAX30102_Reset() != true)
//     {
//         return false;
//     }
//     uint8_t uch_dummy;
//     MAX30102_Read_Reg(REG_INTR_STATUS_1, &uch_dummy);

//     // 3. 检查设备ID，确保通信正常且设备正确
//     if (MAX30102_Read_Reg(REG_PART_ID, &part_id) != true)
//     {
//         return false;
//     }
//     if (part_id != 0x15)
//     {
//         // 如果ID不匹配，返回错误
//         return false;
//     }

//     // 4. 配置传感器参数 (这些参数与您原驱动中的配置相同)

//     // 中断使能: 使能A_FULL (FIFO Almost Full) 和 PPG_RDY (New FIFO Data Ready)
//     MAX30102_Write_Reg(REG_INTR_ENABLE_1, 0xC0);
//     MAX30102_Write_Reg(REG_INTR_ENABLE_2, 0x00);

//     // FIFO指针初始化
//     MAX30102_Write_Reg(REG_FIFO_WR_PTR, 0x00);
//     MAX30102_Write_Reg(REG_OVF_COUNTER, 0x00);
//     MAX30102_Write_Reg(REG_FIFO_RD_PTR, 0x00);

//     // FIFO配置
//     // Sample Averaging = 4, FIFO Rollover = No, FIFO Almost Full = 15
//     // 0x2F 表示采样平均为4次，不翻转，FIFO满中断阈值为 32-15=17
// 	//B7-B5 (SMP_AVE[2:0]): 样本平均设置(1,2,4,8,16,32次)
// 	//B4 (FIFO_ROLLOVER_EN): FIFO满时上卷使能
// 	//B3-B0 (FIFO_A_FULL[3:0]): FIFO几乎满阈值设置
//     MAX30102_Write_Reg(REG_FIFO_CONFIG, 0xAF); // 建议值：4次平均，满阈值为15 (010 1 111)

//     // 模式配置: SpO2 模式
//     MAX30102_Write_Reg(REG_MODE_CONFIG, 0x03);

//     // SpO2配置
//     // ADC Range = 4096nA, Sample Rate = 100Hz, Pulse Width = 411us (18-bit)
//     // 0x27 -> ADC范围4096nA, 采样率100Hz, 脉宽411us
//     MAX30102_Write_Reg(REG_SPO2_CONFIG, 0x2F);

//     // LED脉冲幅度设置 (0x24 约等于 7mA)
//     MAX30102_Write_Reg(REG_LED1_PA, 0x24); // Red LED
//     MAX30102_Write_Reg(REG_LED2_PA, 0x24); // IR LED
//     MAX30102_Write_Reg(REG_PILOT_PA, 0x7f); // Proximity

//     return true;
// }
// ... existing code ...
bool MAX30102_Init(void)
{
    bool transfer_succeeded = true;
    uint8_t part_id = 0;

    // 保存I2C句柄到全局变量
    // max30102_i2c_handle = &hi2c1;  // 假设使用全局I2C句柄变量

    // 软复位传感器
    transfer_succeeded &= MAX30102_Reset();
    if (!transfer_succeeded)
    {
        return false;
    }

    // 清除中断状态寄存器
    uint8_t uch_dummy;
    transfer_succeeded &= MAX30102_Read_Reg(REG_INTR_STATUS_1, &uch_dummy, 1);

    // 验证MAX30102 ID
    transfer_succeeded &= MAX30102_Read_Reg(REG_PART_ID, &part_id,1);
    if (part_id != 0x15)
    {
        return false;
    }

    // 配置中断使能寄存器
    transfer_succeeded &= MAX30102_Write_Reg(REG_INTR_ENABLE_1, 0xC0);
    transfer_succeeded &= MAX30102_Write_Reg(REG_INTR_ENABLE_2, 0x00);

    // 配置FIFO指针寄存器
    transfer_succeeded &= MAX30102_Write_Reg(REG_FIFO_WR_PTR, 0x00);
    transfer_succeeded &= MAX30102_Write_Reg(REG_OVF_COUNTER, 0x00);
    transfer_succeeded &= MAX30102_Write_Reg(REG_FIFO_RD_PTR, 0x00);

    // 配置FIFO设置寄存器
    transfer_succeeded &= MAX30102_Write_Reg(REG_FIFO_CONFIG, 0xAF);

    // 配置模式寄存器
    transfer_succeeded &= MAX30102_Write_Reg(REG_MODE_CONFIG, 0x03);

    // 配置SpO2设置寄存器
    transfer_succeeded &= MAX30102_Write_Reg(REG_SPO2_CONFIG, 0x2F);

    // 配置LED脉冲幅度寄存器
    transfer_succeeded &= MAX30102_Write_Reg(REG_LED1_PA, 0x24);
    transfer_succeeded &= MAX30102_Write_Reg(REG_LED2_PA, 0x24);
    transfer_succeeded &= MAX30102_Write_Reg(REG_PILOT_PA, 0x7f);

    return transfer_succeeded;
}
// ... existing code ...

/**
 * @brief  从MAX30102的FIFO中读取一组数据（Red LED 和 IR LED）
 * @note   每个通道的数据是18位的，存储在32位无符号整型中
 * @param  pun_red_led: 指向存储Red LED数据的变量指针
 * @param  pun_ir_led: 指向存储IR LED数据的变量指针
 * @retval HAL_StatusTypeDef: HAL操作状态
 */
// HAL_StatusTypeDef MAX30102_Read_FIFO(uint32_t *pun_red_led, uint32_t *pun_ir_led)
// {
//     uint8_t buffer[6]; // 用于存储6个字节的FIFO数据 (3字节Red + 3字节IR)

//     // 从FIFO数据寄存器连续读取6个字节
//     if (HAL_I2C_Mem_Read(max30102_i2c_handle, MAX30102_I2C_ADDR, REG_FIFO_DATA, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY) != true)
//     {
//         return false;
//     }

//     // 将3字节数据合成为一个32位整数
//     *pun_red_led = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
//     *pun_ir_led  = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | ((uint32_t)buffer[5]);

//     // 屏蔽掉无效位，MAX30102的ADC分辨率为18位
//     *pun_red_led &= 0x03FFFF;
//     *pun_ir_led  &= 0x03FFFF;

//     return true;
// }

bool MAX30102_Read_FIFO(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
		uint8_t buffer[6]; // 用于存储6个字节的FIFO数据 (3字节Red + 3字节IR)

    // 从FIFO数据寄存器连续读取6个字节
    if (MAX30102_Read_Reg(REG_FIFO_DATA, buffer, 6) != true)
    {
        return false;
    }

    // 将3字节数据合成为一个32位整数
    *pun_red_led = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
    *pun_ir_led  = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | ((uint32_t)buffer[5]);

    // 屏蔽掉无效位，MAX30102的ADC分辨率为18位
    *pun_red_led &= 0x03FFFF;
    *pun_ir_led  &= 0x03FFFF;

    return true;
}
#endif
