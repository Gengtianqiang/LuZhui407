#include "imu_output.h"
#include "icm20948_app.h"
#include "stm32f4xx_hal.h"
#include "Vofa.h"
#include "freertos.h"
#include "task.h"


imu_t       myimu;

void   imu_updata(imu_t *myimu)
{
    // linear_acc
    myimu->linear_acc.x = get_acc_x_current();
    // euler
    myimu->euler = get_euler();
    // euler_yaw_Cali
    myimu->euler_yaw_Cali = get_euler_yaw_Cali();
    
}


//#define I2C_EVENT_MASTER_MODE_SELECT               0x00030001
//#define I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED 0x00070082
//#define I2C_EVENT_MASTER_BYTE_TRANSMITTED          0x00070084
//#define I2C_Direction_Transmitter   ((uint8_t)0x00)
//#define I2C_Direction_Receiver      ((uint8_t)0x01)

//static inline uint32_t GetTick_ms(void)
//{
//    return xTaskGetTickCount() * portTICK_PERIOD_MS;
//}
////typedef enum {RESET = 0, SET = !RESET} FlagStatus;

//FlagStatus I2C_GetFlagStatus(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG)
//{
//    if ((I2Cx->SR1 & I2C_FLAG) != (uint32_t)RESET)
//        return SET;

//    if ((I2Cx->SR2 & I2C_FLAG) != (uint32_t)RESET)
//        return SET;

//    return RESET;
//}
//ErrorStatus I2C_CheckEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
//{
//    uint32_t lastevent;

//    /* 读取 SR1 + SR2 */
//    lastevent = I2Cx->SR1;
//    lastevent |= (uint32_t)(I2Cx->SR2 << 16);

//    if ((lastevent & I2C_EVENT) == I2C_EVENT)
//        return SUCCESS;
//    else
//        return ERROR;
//}
//void I2C_GenerateSTART(I2C_TypeDef *I2Cx, FunctionalState NewState)
//{
//    if (NewState != DISABLE)
//        I2Cx->CR1 |= I2C_CR1_START;
//    else
//        I2Cx->CR1 &= ~I2C_CR1_START;
//}

//void I2C_GenerateSTOP(I2C_TypeDef *I2Cx, FunctionalState NewState)
//{
//    if (NewState != DISABLE)
//        I2Cx->CR1 |= I2C_CR1_STOP;
//    else
//        I2Cx->CR1 &= ~I2C_CR1_STOP;
//}
//void I2C_Send7bitAddress(I2C_TypeDef *I2Cx,
//                         uint8_t Address,
//                         uint8_t I2C_Direction)
//{
//    /* 清 ADDR 位 */
//    (void)I2Cx->SR1;
//    (void)I2Cx->SR2;

//    if (I2C_Direction == I2C_Direction_Transmitter)
//        I2Cx->DR = (uint8_t)(Address & ~0x01);
//    else
//        I2Cx->DR = (uint8_t)(Address | 0x01);
//}
//void I2C_SendData(I2C_TypeDef *I2Cx, uint8_t Data)
//{
//    I2Cx->DR = Data;
//}

//uint8_t I2C_ReceiveData(I2C_TypeDef *I2Cx)
//{
//    return (uint8_t)I2Cx->DR;
//}
//void I2C_SoftwareResetCmd(I2C_TypeDef *I2Cx, FunctionalState NewState)
//{
//    if (NewState != DISABLE)
//        I2Cx->CR1 |= I2C_CR1_SWRST;
//    else
//        I2Cx->CR1 &= ~I2C_CR1_SWRST;
//}

//uint8_t my_I2C_Mem_Write_Std(I2C_TypeDef *I2Cx,
//                                   uint16_t DevAddress,
//                                   uint16_t MemAddress,
//                                   uint16_t MemAddSize,
//                                   uint8_t *pData,
//                                   uint16_t Size,
//                                   uint32_t Timeout)
//{
//    uint32_t tick = GetTick_ms();

//    /* 等待总线空闲 */
//    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
//        if (GetTick_ms() - tick > Timeout)
//            return 2;

//    /* START */
//    I2C_GenerateSTART(I2Cx, ENABLE);
//    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
//        if (GetTick_ms() - tick > Timeout)
//            return 3;

//    /* 发送从机地址（写） */
//    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Transmitter);
//    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//        if (GetTick_ms() - tick > Timeout)
//            return 3;

//    /* 发送内存地址 */
//    if (MemAddSize == 2)   /* 16bit */
//    {
//        I2C_SendData(I2Cx, (uint8_t)(MemAddress >> 8));
//        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//        I2C_SendData(I2Cx, (uint8_t)(MemAddress));
//        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//    }
//    else                  /* 8bit */
//    {
//        I2C_SendData(I2Cx, (uint8_t)(MemAddress));
//        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//    }

//    /* 发送数据 */
//    while (Size--)
//    {
//        I2C_SendData(I2Cx, *pData++);
//        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//            if (GetTick_ms() - tick > Timeout)
//                return 3;
//    }

//    /* STOP */
//    I2C_GenerateSTOP(I2Cx, ENABLE);

//    return 0;
//}
