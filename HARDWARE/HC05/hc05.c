/**
  ******************************************************************************
  * @file    hc05.c
  * @brief   HC-05 蓝牙模块驱动（纯硬件驱动，只负责收发和指令解析）
  ******************************************************************************
  */

#include "hc05.h"
#include "usart.h"
#include "string.h"
#include "stdlib.h"
#include "delay.h"

/*============================================================================
 * 静态函数声明
 *============================================================================*/
static char        GetFirstValidChar(char *cmd);
static HC05_Cmd_t  ParseSingleChar(char c);
static HC05_Cmd_t  ParseStringCommand(char *cmd);

/*============================================================================
 * 公共函数实现
 *============================================================================*/

/**
 * @brief  初始化 HC-05 蓝牙模块
 */
void HC05_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* STATE 引脚（PA8）- 检测连接状态 */
    RCC_APB2PeriphClockCmd(HC05_STATE_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = HC05_STATE_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HC05_STATE_PORT, &GPIO_InitStructure);
    
    /* EN/KEY 引脚（PA11）- 控制透传模式 */
    RCC_APB2PeriphClockCmd(HC05_EN_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = HC05_EN_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HC05_EN_PORT, &GPIO_InitStructure);
    
    HC05_EXIT_AT();          /* 进入透传模式 */
    delay_ms(500);           /* 等待模块稳定 */
}

/**
 * @brief  通过蓝牙发送字符串
 * @param  str  要发送的字符串
 */
void HC05_SendString(char *str)
{
    while (*str)
    {
        while ((USART1->SR & 0x40) == 0);
        USART1->DR = *str++;
    }
}

/**
 * @brief  获取字符串中第一个有效字符（跳过回车换行和空格）
 * @param  cmd  字符串
 * @return 第一个有效字符，没有则返回 0
 */
static char GetFirstValidChar(char *cmd)
{
    for (int i = 0; cmd[i] != '\0'; i++)
    {
        if (cmd[i] != '\r' && cmd[i] != '\n' && cmd[i] != ' ')
            return cmd[i];
    }
    return 0;
}

/**
 * @brief  解析单字符指令
 * @param  c  单字符
 * @return 指令类型
 */
static HC05_Cmd_t ParseSingleChar(char c)
{
    if (c >= '0' && c <= '4')
        return (HC05_Cmd_t)(HC05_CMD_GEAR_0 + (c - '0'));
    
    if (c == 'A' || c == 'a') return HC05_CMD_AUTO;
    if (c == 'M' || c == 'm') return HC05_CMD_MANUAL;
    if (c == 'X' || c == 'x' || c == 'E' || c == 'e') return HC05_CMD_EXIT;
    
    return HC05_CMD_NONE;
}

/**
 * @brief  解析字符串指令
 * @param  cmd  字符串
 * @return 指令类型
 */
static HC05_Cmd_t ParseStringCommand(char *cmd)
{
    if (strstr(cmd, "EXIT")   != NULL) return HC05_CMD_EXIT;
    if (strstr(cmd, "MANUAL") != NULL) return HC05_CMD_MANUAL;
    if (strstr(cmd, "AUTO")   != NULL) return HC05_CMD_AUTO;
    if (strstr(cmd, "TEMP:")  != NULL) return HC05_CMD_SET_TEMP;
    if (strstr(cmd, "GEAR:")  != NULL) return HC05_CMD_SET_GEAR;
    
    return HC05_CMD_NONE;
}

/**
 * @brief  解析蓝牙串口指令（纯解析，不执行业务逻辑）
 * @param  buf  接收缓冲区
 * @return 解析结果（指令类型 + 附带值）
 */
HC05_Result_t HC05_ParseCommand(uint8_t *buf)
{
    HC05_Result_t result;
    result.cmd   = HC05_CMD_NONE;
    result.value = 0.0f;
    
    char *cmd       = (char *)buf;
    char  firstChar = GetFirstValidChar(cmd);
    char *p;
    
    /* 单字符指令优先 */
    if (firstChar != 0)
    {
        result.cmd = ParseSingleChar(firstChar);
        if (result.cmd >= HC05_CMD_GEAR_0 && result.cmd <= HC05_CMD_GEAR_4)
            result.value = (float)(result.cmd - HC05_CMD_GEAR_0);
        return result;
    }
    
    /* 字符串指令 */
    result.cmd = ParseStringCommand(cmd);
    
    if (result.cmd == HC05_CMD_SET_TEMP)
    {
        p = strstr(cmd, "TEMP:");
        if (p != NULL) result.value = atof(p + 5);
    }
    else if (result.cmd == HC05_CMD_SET_GEAR)
    {
        p = strstr(cmd, "GEAR:");
        if (p != NULL) result.value = (float)atoi(p + 5);
    }
    
    return result;
}