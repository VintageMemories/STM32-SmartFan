/**
  ******************************************************************************
  * @ 文件   led.c
  * @ 描述   LED 指示灯驱动
  ******************************************************************************
  */

#include "led.h"

/*============================================================================
 * 宏定义
 *============================================================================*/
#define SLOW_HALF_PERIOD    5   /* 250ms (5 * 50ms) */
#define FAST_HALF_PERIOD    2   /* 100ms (2 * 50ms) */

/*============================================================================
 * 静态变量
 *============================================================================*/
static LED_Mode_t ledMode = LED_MODE_OFF;
static uint8_t  ledState = 0;
static uint16_t ledTick = 0;

/*============================================================================
 * 公共函数实现
 *============================================================================*/

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(LED_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    
    LED_OFF();
    ledMode = LED_MODE_OFF;
    ledState = 0;
    ledTick = 0;
}

void LED_On(void)
{
    LED_ON();
    ledState = 1;
}

void LED_Off(void)
{
    LED_OFF();
    ledState = 0;
}

void LED_Toggle(void)
{
    if (GPIO_ReadOutputDataBit(LED_PORT, LED_PIN))
        LED_Off();
    else
        LED_On();
}

void LED_SetMode(LED_Mode_t mode)
{
    ledMode = mode;
    ledTick = 0;
    
    switch (mode)
    {
        case LED_MODE_OFF:  LED_Off(); break;
        case LED_MODE_ON:   LED_On();  break;
        case LED_MODE_SLOW:
        case LED_MODE_FAST: LED_On();  break;
    }
}

void LED_Update(void)
{
    uint8_t halfPeriod = 0;
    
    if (ledMode == LED_MODE_OFF || ledMode == LED_MODE_ON)
        return;
    
    halfPeriod = (ledMode == LED_MODE_SLOW) ? SLOW_HALF_PERIOD : FAST_HALF_PERIOD;
    
    if (++ledTick >= halfPeriod)
    {
        ledTick = 0;
        if (ledState)
            LED_Off();
        else
            LED_On();
    }
}
