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
 * 蓝牙指令枚举（纯解析结果，不含业务含义）
 *============================================================================*/
typedef enum {
    HC05_CMD_GEAR_0 = 0,    /* 档位 0 */
    HC05_CMD_GEAR_1,        /* 档位 1 */
    HC05_CMD_GEAR_2,        /* 档位 2 */
    HC05_CMD_GEAR_3,        /* 档位 3 */
    HC05_CMD_GEAR_4,        /* 档位 4 */
    HC05_CMD_AUTO,          /* 自动模式 */
    HC05_CMD_MANUAL,        /* 手动模式 */
    HC05_CMD_EXIT,          /* 退出蓝牙 */
    HC05_CMD_SET_TEMP,      /* 设置温度 */
    HC05_CMD_SET_GEAR,      /* 设置档位 */
    HC05_CMD_NONE           /* 无效指令 */
} HC05_Cmd_t;

/*============================================================================
 * 指令解析结果
 *============================================================================*/
typedef struct {
    HC05_Cmd_t  cmd;        /* 指令类型 */
    float       value;      /* 附带值（温度或档位） */
} HC05_Result_t;

/*============================================================================
 * 函数声明
 *============================================================================*/
void HC05_Init(void);
void HC05_SendString(char *str);
HC05_Result_t HC05_ParseCommand(uint8_t *buf);

#endif