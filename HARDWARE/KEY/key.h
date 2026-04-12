#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

// 按键引脚定义
#define KEY1_PORT   GPIOC
#define KEY1_PIN    GPIO_Pin_0
#define KEY1_CLK    RCC_APB2Periph_GPIOC

#define KEY2_PORT   GPIOC
#define KEY2_PIN    GPIO_Pin_1
#define KEY2_CLK    RCC_APB2Periph_GPIOC

// 按键按下检测（低电平有效）
#define KEY1_PRESSED    (GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == Bit_RESET)
#define KEY2_PRESSED    (GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN) == Bit_RESET)

// 按键事件类型
typedef enum {
    KEY_NONE = 0,
    KEY1_SHORT,
    KEY1_LONG,
    KEY2_SHORT,
    KEY2_LONG
} Key_Event_t;

// 函数声明
void KEY_Init(void);
Key_Event_t KEY_Scan(void);

#endif

