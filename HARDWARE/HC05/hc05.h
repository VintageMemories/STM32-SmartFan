#ifndef __HC05_H
#define __HC05_H

#include "sys.h"

/*============================================================================
 * 引脚定义
 *============================================================================*/
/* STATE 引脚（PA8）- 检测蓝牙连接状态 */
#define HC05_STATE_PORT     GPIOA
#define HC05_STATE_PIN      GPIO_Pin_8
#define HC05_STATE_CLK      RCC_APB2Periph_GPIOA

/* EN/KEY 引脚（PA11）- 控制透传模式 */
#define HC05_EN_PORT        GPIOA
#define HC05_EN_PIN         GPIO_Pin_11
#define HC05_EN_CLK         RCC_APB2Periph_GPIOA

/*============================================================================
 * 操作宏
 *============================================================================*/
#define HC05_IS_CONNECTED() (GPIO_ReadInputDataBit(HC05_STATE_PORT, HC05_STATE_PIN) == Bit_SET)
#define HC05_ENTER_AT()     GPIO_SetBits(HC05_EN_PORT, HC05_EN_PIN)
#define HC05_EXIT_AT()      GPIO_ResetBits(HC05_EN_PORT, HC05_EN_PIN)

/*============================================================================
 * 枚举类型定义
 *============================================================================*/
typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL,
    MODE_BLUETOOTH
} SystemMode_t;

typedef enum {
    WORK_MODE_MANUAL = 0,
    WORK_MODE_AUTO
} BTWorkMode_t;

/*============================================================================
 * 全局变量声明
 *============================================================================*/
extern SystemMode_t sysMode;
extern BTWorkMode_t btWorkMode;
extern float currentTemp;
extern float targetTemp;
extern uint8_t manualGear;
extern uint8_t fanDuty;
extern uint8_t btConnected;
extern const uint8_t gearDutyMap[5];

/*============================================================================
 * 函数声明
 *============================================================================*/
void HC05_Init(void);
void HC05_SendString(char *str);
void HC05_ParseCommand(uint8_t *buf);
void HC05_CheckConnection(void);
void PID_Calculate(void);
void SaveConfig(void);
void LoadConfig(void);

#endif

