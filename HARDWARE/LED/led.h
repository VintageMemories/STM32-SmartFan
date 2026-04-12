#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

/*============================================================================
 * 引脚定义
 *============================================================================*/
#define LED_PORT    GPIOA
#define LED_PIN     GPIO_Pin_4
#define LED_CLK     RCC_APB2Periph_GPIOA

/*============================================================================
 * 操作宏（低电平点亮）
 *============================================================================*/
#define LED_ON()    GPIO_ResetBits(LED_PORT, LED_PIN)
#define LED_OFF()   GPIO_SetBits(LED_PORT, LED_PIN)
#define LED_TOGGLE() GPIO_WriteBit(LED_PORT, LED_PIN, \
                      (BitAction)(1 - GPIO_ReadOutputDataBit(LED_PORT, LED_PIN)))

/*============================================================================
 * 枚举类型
 *============================================================================*/
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_ON,
    LED_MODE_SLOW,
    LED_MODE_FAST
} LED_Mode_t;

/*============================================================================
 * 函数声明
 *============================================================================*/
void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_SetMode(LED_Mode_t mode);
void LED_Update(void);

#endif

