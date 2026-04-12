#ifndef __IIC_SHT30_H
#define __IIC_SHT30_H

#include "stm32f10x.h"
#include "delay.h"

#define SHT30_SCLK_PORT     GPIOB
#define SHT30_SCLK_PIN      GPIO_Pin_8
#define SHT30_SDIN_PORT     GPIOB
#define SHT30_SDIN_PIN      GPIO_Pin_9
#define SHT30_I2C_CLK       RCC_APB2Periph_GPIOB

#define SHT30_SCLK_Set()    GPIO_SetBits(SHT30_SCLK_PORT, SHT30_SCLK_PIN)
#define SHT30_SCLK_Clr()    GPIO_ResetBits(SHT30_SCLK_PORT, SHT30_SCLK_PIN)
#define SHT30_SDIN_Set()    GPIO_SetBits(SHT30_SDIN_PORT, SHT30_SDIN_PIN)
#define SHT30_SDIN_Clr()    GPIO_ResetBits(SHT30_SDIN_PORT, SHT30_SDIN_PIN)
#define SHT30_SDIN_Read()   GPIO_ReadInputDataBit(SHT30_SDIN_PORT, SHT30_SDIN_PIN)

#define SHT30_ADDR_WRITE    0x88
#define SHT30_ADDR_READ     0x89

#define SHT30_CMD_SOFT_RESET    0x30A2
#define SHT30_CMD_MEAS_HIGH     0x2C06

void SHT30_SDA_OUT(void);
void SHT30_SDA_IN(void);
void SHT30_IIC_Start(void);
void SHT30_IIC_Stop(void);
void SHT30_IIC_Wait_Ack(void);
void SHT30_Write_IIC_Byte(uint8_t IIC_Byte);
uint8_t SHT30_Read_IIC_Byte(uint8_t ack);
void SHT30_SendCmd(uint16_t cmd);
void SHT30_Init(void);
uint8_t SHT30_ReadData(float *temperature, float *humidity);
float SHT30_ReadTemperature(void);

#endif

