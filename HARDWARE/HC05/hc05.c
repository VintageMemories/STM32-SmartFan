/**
  ******************************************************************************
  * @ 文件   hc05.c
  * @ 描述   HC-05 蓝牙模块驱动
  ******************************************************************************
  */

#include "hc05.h"
#include "usart.h"
#include "led.h"
#include "string.h"
#include "stdlib.h"
#include "delay.h"

/*============================================================================
 * 指令配置宏定义
 *============================================================================*/
#define TEMP_MIN                16.0f   /* 最低可设温度 */
#define TEMP_MAX                35.0f   /* 最高可设温度 */
#define GEAR_MIN                0       /* 最小档位 */
#define GEAR_MAX                4       /* 最大档位 */

#define CMD_AUTO                "AUTO"
#define CMD_MANUAL              "MANUAL"
#define CMD_EXIT                "EXIT"
#define CMD_TEMP                "TEMP:"
#define CMD_GEAR                "GEAR:"

#define TEMP_OFFSET             5       /* strlen("TEMP:") */
#define GEAR_OFFSET             5       /* strlen("GEAR:") */

/*============================================================================
 * 全局变量定义
 *============================================================================*/
SystemMode_t sysMode = MODE_AUTO;
BTWorkMode_t btWorkMode = WORK_MODE_MANUAL;
float currentTemp = 25.0f;
float targetTemp = 26.0f;
uint8_t manualGear = 2;
uint8_t fanDuty = 0;
uint8_t btConnected = 0;

const uint8_t gearDutyMap[5] = {0, 30, 50, 80, 100};

/* PID 变量（保留，供其他模块使用）*/
float pidOutput = 0;
float lastError = 0;
float prevError = 0;
float Kp = 6.0f;
float Ki = 0.05f;
float Kd = 0.8f;

/*============================================================================
 * 静态函数声明
 *============================================================================*/
static char GetFirstValidChar(char *cmd);
static void HandleSingleChar(char c);
static void HandleStringCommand(char *cmd);

/*============================================================================
 * 公共函数实现
 *============================================================================*/

/**
 * @brief 初始化 HC-05 蓝牙模块
 */
void HC05_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* STATE 引脚（PA8）- 检测连接状态 */
    RCC_APB2PeriphClockCmd(HC05_STATE_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = HC05_STATE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HC05_STATE_PORT, &GPIO_InitStructure);
    
    /* EN/KEY 引脚（PA11）- 控制透传模式 */
    RCC_APB2PeriphClockCmd(HC05_EN_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = HC05_EN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HC05_EN_PORT, &GPIO_InitStructure);
    
    HC05_EXIT_AT();          /* 进入透传模式 */
    delay_ms(500);           /* 等待模块稳定 */
    btConnected = HC05_IS_CONNECTED();
}

/**
 * @brief 通过蓝牙发送字符串
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
 * @brief 获取字符串中第一个有效字符（跳过回车换行和空格）
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
 * @brief 处理单字符指令
 */
static void HandleSingleChar(char c)
{
    extern void Fan_SetGear(uint8_t gear);
    
    /* 档位指令 0-4 */
    if (c >= '0' && c <= '4')
    {
        manualGear = c - '0';
        fanDuty = gearDutyMap[manualGear];
        SaveConfig();
        Fan_SetGear(manualGear);
        
        char ack[16];
        sprintf(ack, "Gear:%d\r\n", manualGear);
        HC05_SendString(ack);
        return;
    }
    
    /* 自动模式 A/a */
    if (c == 'A' || c == 'a')
    {
        if (sysMode == MODE_BLUETOOTH)
            btWorkMode = WORK_MODE_AUTO;
        HC05_SendString("Auto\r\n");
        return;
    }
    
    /* 手动模式 M/m */
    if (c == 'M' || c == 'm')
    {
        if (sysMode == MODE_BLUETOOTH)
            btWorkMode = WORK_MODE_MANUAL;
        HC05_SendString("Manual\r\n");
        return;
    }
    
    /* 退出蓝牙模式 X/x/E/e */
    if (c == 'X' || c == 'x' || c == 'E' || c == 'e')
    {
        sysMode = MODE_MANUAL;
        btWorkMode = WORK_MODE_MANUAL;
        LED_SetMode(LED_MODE_ON);
        HC05_SendString("Exit\r\n");
    }
}

/**
 * @brief 处理字符串指令
 */
static void HandleStringCommand(char *cmd)
{
    extern void Fan_SetGear(uint8_t gear);
    char *p;
    
    /* 退出蓝牙模式 */
    if (strstr(cmd, CMD_EXIT) != NULL)
    {
        sysMode = MODE_MANUAL;
        btWorkMode = WORK_MODE_MANUAL;
        LED_SetMode(LED_MODE_ON);
        HC05_SendString("Exit\r\n");
        return;
    }
    
    /* 以下指令仅在蓝牙模式下有效 */
    if (sysMode != MODE_BLUETOOTH) return;
    
    /* 手动模式 */
    if (strstr(cmd, CMD_MANUAL) != NULL)
    {
        btWorkMode = WORK_MODE_MANUAL;
        HC05_SendString("Manual\r\n");
        return;
    }
    
    /* 自动模式 */
    if (strstr(cmd, CMD_AUTO) != NULL)
    {
        btWorkMode = WORK_MODE_AUTO;
        HC05_SendString("Auto\r\n");
        return;
    }
    
    /* 设置目标温度 */
    p = strstr(cmd, CMD_TEMP);
    if (p != NULL)
    {
        float temp = atof(p + TEMP_OFFSET);
        if (temp >= TEMP_MIN && temp <= TEMP_MAX)
        {
            targetTemp = temp;
            SaveConfig();
            char ack[32];
            sprintf(ack, "Target:%.1f\r\n", temp);
            HC05_SendString(ack);
        }
        return;
    }
    
    /* 设置档位 */
    p = strstr(cmd, CMD_GEAR);
    if (p != NULL)
    {
        int gear = atoi(p + GEAR_OFFSET);
        if (gear >= GEAR_MIN && gear <= GEAR_MAX)
        {
            manualGear = gear;
            fanDuty = gearDutyMap[gear];
            SaveConfig();
            Fan_SetGear(manualGear);
            char ack[32];
            sprintf(ack, "Gear:%d\r\n", gear);
            HC05_SendString(ack);
        }
    }
}

/**
 * @brief 解析蓝牙串口指令
 */
void HC05_ParseCommand(uint8_t *buf)
{
    char *cmd = (char *)buf;
    char firstChar = GetFirstValidChar(cmd);
    
    if (firstChar != 0)
        HandleSingleChar(firstChar);
    
    HandleStringCommand(cmd);
}

/**
 * @brief 检测蓝牙连接状态变化
 */
void HC05_CheckConnection(void)
{
    static uint8_t lastState = 0;
    uint8_t curState = HC05_IS_CONNECTED();
    
    if (curState != lastState)
    {
        lastState = curState;
        btConnected = curState;
        
        if (sysMode == MODE_BLUETOOTH)
        {
            LED_SetMode(btConnected ? LED_MODE_ON : LED_MODE_FAST);
        }
    }
}

/**
 * @brief PID 计算（保留，供其他模块调用）
 */
void PID_Calculate(void)
{
    float error = targetTemp - currentTemp;
    
    pidOutput += Kp * (error - lastError)
               + Ki * error
               + Kd * (error - 2 * lastError + prevError);
    
    if (pidOutput > 100.0f) pidOutput = 100.0f;
    if (pidOutput < 0.0f)   pidOutput = 0.0f;
    
    fanDuty = (uint8_t)pidOutput;
    
    prevError = lastError;
    lastError = error;
}

/**
 * @brief 保存配置到备份寄存器
 */
void SaveConfig(void)
{
    uint16_t tempInt = (uint16_t)(targetTemp * 10);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    
    BKP_WriteBackupRegister(BKP_DR1, tempInt);
    BKP_WriteBackupRegister(BKP_DR2, (uint16_t)sysMode);
    BKP_WriteBackupRegister(BKP_DR3, (uint16_t)manualGear);
}

/**
 * @brief 从备份寄存器加载配置
 */
void LoadConfig(void)
{
    uint16_t tempInt;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    
    tempInt = BKP_ReadBackupRegister(BKP_DR1);
    if (tempInt >= 160 && tempInt <= 350)
    {
        targetTemp = tempInt / 10.0f;
        sysMode = (SystemMode_t)BKP_ReadBackupRegister(BKP_DR2);
        manualGear = (uint8_t)BKP_ReadBackupRegister(BKP_DR3);
        
        if (manualGear > 4) manualGear = 2;
        if (sysMode > MODE_BLUETOOTH) sysMode = MODE_AUTO;
    }
}
