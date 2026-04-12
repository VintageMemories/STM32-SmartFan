/**
  ******************************************************************************
  * @ 文件   exti.c
  * @ 描述   外部中断按键处理
  ******************************************************************************
  */

#include "exti.h"
#include "hc05.h"
#include "led.h"
#include "delay.h"

/*============================================================================
 * 宏定义
 *============================================================================*/
#define LONG_PRESS_TIME_MS      2000    /* 长按触发时间 2s */

/* 按键引脚 */
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
 * 外部变量引用
 *============================================================================*/
extern uint8_t manualGear;
extern SystemMode_t sysMode;
extern BTWorkMode_t btWorkMode;
extern volatile uint32_t g_msTicks;

/*============================================================================
 * 静态变量
 *============================================================================*/
static uint32_t mode_key_press_time = 0;
static uint8_t  mode_key_pressed = 0;
static uint8_t  mode_key_long_triggered = 0;

/*============================================================================
 * 公共函数实现
 *============================================================================*/

/**
 * @brief 初始化按键外部中断
 */
void EXTI_KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(GEAR_KEY_CLK | MODE_KEY_CLK | RCC_APB2Periph_AFIO, ENABLE);

    /* 配置为上拉输入 */
    GPIO_InitStructure.GPIO_Pin = GEAR_KEY_PIN | MODE_KEY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GEAR_KEY_PORT, &GPIO_InitStructure);

    /* 连接 EXTI 线路 */
    GPIO_EXTILineConfig(GEAR_KEY_PORT_SRC, GEAR_KEY_PIN_SRC);
    GPIO_EXTILineConfig(MODE_KEY_PORT_SRC, MODE_KEY_PIN_SRC);

    /* 配置下降沿触发 */
    EXTI_InitStructure.EXTI_Line = GEAR_KEY_EXTI_LINE | MODE_KEY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 档位按键中断 */
    NVIC_InitStructure.NVIC_IRQChannel = GEAR_KEY_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 模式按键中断 */
    NVIC_InitStructure.NVIC_IRQChannel = MODE_KEY_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief 档位按键中断服务函数（PC0）
 */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(GEAR_KEY_EXTI_LINE) != RESET)
    {
        /* 手动模式或蓝牙手动模式下可换档 */
        if (sysMode == MODE_MANUAL || 
            (sysMode == MODE_BLUETOOTH && btWorkMode == WORK_MODE_MANUAL))
        {
            manualGear = (manualGear + 1) % 5;
        }
        EXTI_ClearITPendingBit(GEAR_KEY_EXTI_LINE);
    }
}

/**
 * @brief 模式按键中断服务函数（PC1）
 */
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(MODE_KEY_EXTI_LINE) != RESET)
    {
        mode_key_press_time = g_msTicks;
        mode_key_pressed = 1;
        mode_key_long_triggered = 0;
        EXTI_ClearITPendingBit(MODE_KEY_EXTI_LINE);
    }
}

/**
 * @brief 模式按键状态处理（主循环调用）
 */
void Mode_Key_Process(void)
{
    if (!mode_key_pressed) return;
    
    uint8_t key_state = GPIO_ReadInputDataBit(MODE_KEY_PORT, MODE_KEY_PIN);
    uint32_t press_duration = g_msTicks - mode_key_press_time;
    
    if (key_state == Bit_SET)  /* 按键释放 */
    {
        if (!mode_key_long_triggered && press_duration < LONG_PRESS_TIME_MS)
        {
            /* 短按：非蓝牙模式下切换手动/自动 */
            if (sysMode != MODE_BLUETOOTH)
            {
                sysMode = (sysMode == MODE_MANUAL) ? MODE_AUTO : MODE_MANUAL;
            }
        }
        mode_key_pressed = 0;
    }
    else  /* 按键按下中 */
    {
        if (!mode_key_long_triggered && press_duration >= LONG_PRESS_TIME_MS)
        {
            mode_key_long_triggered = 1;
            
            if (sysMode == MODE_BLUETOOTH)  /* 退出蓝牙模式 */
            {
                sysMode = (btWorkMode == WORK_MODE_MANUAL) ? MODE_MANUAL : MODE_AUTO;
                LED_SetMode(LED_MODE_ON);
            }
            else  /* 进入蓝牙模式 */
            {
                btWorkMode = (sysMode == MODE_MANUAL) ? WORK_MODE_MANUAL : WORK_MODE_AUTO;
                sysMode = MODE_BLUETOOTH;
                LED_SetMode(HC05_IS_CONNECTED() ? LED_MODE_ON : LED_MODE_FAST);
            }
        }
    }
}
