/**
  ******************************************************************************
  * @file    exti.c
  * @brief   外部中断初始化
  ******************************************************************************
  */

#include "exti.h"

/*============================================================================
 * 引脚定义
 *============================================================================*/
#define GEAR_KEY_PORT           GPIOC
#define GEAR_KEY_PIN            GPIO_Pin_0
#define GEAR_KEY_CLK            RCC_APB2Periph_GPIOC
#define GEAR_KEY_EXTI_LINE      EXTI_Line0
#define GEAR_KEY_PORT_SRC       GPIO_PortSourceGPIOC
#define GEAR_KEY_PIN_SRC        GPIO_PinSource0
#define GEAR_KEY_IRQn           EXTI0_IRQn

#define MODE_KEY_PORT           GPIOC
#define MODE_KEY_PIN            GPIO_Pin_1
#define MODE_KEY_CLK            RCC_APB2Periph_GPIOC
#define MODE_KEY_EXTI_LINE      EXTI_Line1
#define MODE_KEY_PORT_SRC       GPIO_PortSourceGPIOC
#define MODE_KEY_PIN_SRC        GPIO_PinSource1
#define MODE_KEY_IRQn           EXTI1_IRQn

/*============================================================================
 * 公共函数实现
 *============================================================================*/

/**
 * @brief  初始化按键外部中断
 */
void EXTI_KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(GEAR_KEY_CLK | MODE_KEY_CLK | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin  = GEAR_KEY_PIN | MODE_KEY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GEAR_KEY_PORT, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GEAR_KEY_PORT_SRC, GEAR_KEY_PIN_SRC);
    GPIO_EXTILineConfig(MODE_KEY_PORT_SRC, MODE_KEY_PIN_SRC);

    EXTI_InitStructure.EXTI_Line    = GEAR_KEY_EXTI_LINE | MODE_KEY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = GEAR_KEY_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = MODE_KEY_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*============================================================================
 * 中断服务函数
 *============================================================================*/
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(GEAR_KEY_EXTI_LINE) != RESET)
        EXTI_ClearITPendingBit(GEAR_KEY_EXTI_LINE);
}

void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(MODE_KEY_EXTI_LINE) != RESET)
        EXTI_ClearITPendingBit(MODE_KEY_EXTI_LINE);
}