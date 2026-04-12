#include "iic_sht30.h"

static uint8_t SHT30_CRC8(const uint8_t *data, uint8_t len);

void SHT30_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SHT30_SDIN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SHT30_SDIN_PORT, &GPIO_InitStructure);
}

void SHT30_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SHT30_SDIN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(SHT30_SDIN_PORT, &GPIO_InitStructure);
}

void SHT30_IIC_Start(void)
{
    SHT30_SDA_OUT();
    SHT30_SCLK_Set();
    SHT30_SDIN_Set();
    SHT30_SDIN_Clr();
    SHT30_SCLK_Clr();
}

void SHT30_IIC_Stop(void)
{
    SHT30_SDA_OUT();
    SHT30_SCLK_Set();
    SHT30_SDIN_Clr();
    SHT30_SDIN_Set();
}

void SHT30_IIC_Wait_Ack(void)
{
    SHT30_SCLK_Set();
    SHT30_SCLK_Clr();
}

void SHT30_Write_IIC_Byte(uint8_t IIC_Byte)
{
    uint8_t i;
    SHT30_SDA_OUT();
    SHT30_SCLK_Clr();
    for (i = 0; i < 8; i++)
    {
        if (IIC_Byte & 0x80)
            SHT30_SDIN_Set();
        else
            SHT30_SDIN_Clr();
        IIC_Byte <<= 1;
        SHT30_SCLK_Set();
        SHT30_SCLK_Clr();
    }
}

uint8_t SHT30_Read_IIC_Byte(uint8_t ack)
{
    uint8_t i, data = 0;
    SHT30_SDA_IN();
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        SHT30_SCLK_Set();
        if (SHT30_SDIN_Read())
            data |= 0x01;
        SHT30_SCLK_Clr();
    }
    SHT30_SDA_OUT();
    if (ack)
        SHT30_SDIN_Set();
    else
        SHT30_SDIN_Clr();
    SHT30_SCLK_Set();
    SHT30_SCLK_Clr();
    return data;
}

void SHT30_SendCmd(uint16_t cmd)
{
    SHT30_IIC_Start();
    SHT30_Write_IIC_Byte(SHT30_ADDR_WRITE);
    SHT30_IIC_Wait_Ack();
    SHT30_Write_IIC_Byte(cmd >> 8);
    SHT30_IIC_Wait_Ack();
    SHT30_Write_IIC_Byte(cmd & 0xFF);
    SHT30_IIC_Wait_Ack();
    SHT30_IIC_Stop();
}

static uint8_t SHT30_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void SHT30_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(SHT30_I2C_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = SHT30_SCLK_PIN | SHT30_SDIN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SHT30_SCLK_PORT, &GPIO_InitStructure);
    SHT30_SCLK_Set();
    SHT30_SDIN_Set();
    delay_ms(10);
    SHT30_SendCmd(SHT30_CMD_SOFT_RESET);
    delay_ms(20);
}

uint8_t SHT30_ReadData(float *temperature, float *humidity)
{
    uint8_t data[6];
    uint16_t tempRaw, humRaw;
    uint8_t i;

    SHT30_SendCmd(SHT30_CMD_MEAS_HIGH);
    delay_ms(20);

    SHT30_IIC_Start();
    SHT30_Write_IIC_Byte(SHT30_ADDR_READ);
    SHT30_IIC_Wait_Ack();
    for (i = 0; i < 6; i++)
    {
        data[i] = SHT30_Read_IIC_Byte(i == 5 ? 1 : 0);
    }
    SHT30_IIC_Stop();

    if (SHT30_CRC8(&data[0], 2) != data[2]) return 1;
    if (SHT30_CRC8(&data[3], 2) != data[5]) return 1;

    tempRaw = (data[0] << 8) | data[1];
    *temperature = -45.0f + 175.0f * ((float)tempRaw / 65535.0f);

    humRaw = (data[3] << 8) | data[4];
    *humidity = 100.0f * ((float)humRaw / 65535.0f);

    return 0;
}

float SHT30_ReadTemperature(void)
{
    float temp, hum;
    if (SHT30_ReadData(&temp, &hum) == 0)
        return temp;
    else
        return -99.0f;
}
