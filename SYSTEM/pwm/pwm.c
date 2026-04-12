/**
  ******************************************************************************
  * @ 文件   pwm.c
  * @ 描述   TIM3 PWM 输出驱动（风扇调速）
  ******************************************************************************
  */

#include "pwm.h"

/*============================================================================
 * 宏定义
 *============================================================================*/
#define PWM_PERIOD      1000    /* ARR = 1000 */
#define PWM_PRESCALER   72      /* 72MHz / 72 = 1MHz */
#define PWM_GPIO_PIN    GPIO_Pin_6
#define PWM_GPIO_PORT   GPIOA

void PWM_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* PA6 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = PWM_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStructure);

    /* TIM3 时基配置 */
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD - 1;      /* ARR = 999 */
    TIM_TimeBaseStructure.TIM_Prescaler = PWM_PRESCALER - 1; /* PSC = 71 */
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWM 模式配置 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}
