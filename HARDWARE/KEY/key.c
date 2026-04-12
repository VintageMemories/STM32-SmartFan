#include "key.h"

// 按键消抖及长按时间阈值（单位：ms）
#define SHORT_PRESS_MIN     20      // 最短有效按下时间
#define LONG_PRESS_TIME     2000    // 长按触发时间

// 按键状态
static uint8_t  key1_last = 1;      // 上次电平，1=释放
static uint8_t  key2_last = 1;
static uint32_t key1_press_time = 0;
static uint32_t key2_press_time = 0;
static uint8_t  key1_long_flag = 0;
static uint8_t  key2_long_flag = 0;

// 外部时间变量，在 SysTick 中断中自增
extern volatile uint32_t g_msTicks;

void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(KEY1_CLK | KEY2_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN | KEY2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 上拉输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

Key_Event_t KEY_Scan(void)
{
    Key_Event_t event = KEY_NONE;
    uint8_t key1_cur = KEY1_PRESSED ? 0 : 1;   // 按下为 0
    uint8_t key2_cur = KEY2_PRESSED ? 0 : 1;
    uint32_t now = g_msTicks;
    
    // 按键1 处理
    if (key1_cur == 0 && key1_last == 1)        // 按下
    {
        key1_press_time = now;
        key1_long_flag = 0;
    }
    else if (key1_cur == 0 && key1_last == 0)   // 按住中
    {
        if (key1_long_flag == 0 && (now - key1_press_time >= LONG_PRESS_TIME))
        {
            key1_long_flag = 1;
            event = KEY1_LONG;
        }
    }
    else if (key1_cur == 1 && key1_last == 0)   // 释放
    {
        if (key1_long_flag == 0 && (now - key1_press_time >= SHORT_PRESS_MIN))
        {
            event = KEY1_SHORT;
        }
    }
    
    // 按键2 处理
    if (key2_cur == 0 && key2_last == 1)        // 按下
    {
        key2_press_time = now;
        key2_long_flag = 0;
    }
    else if (key2_cur == 0 && key2_last == 0)   // 按住中
    {
        if (key2_long_flag == 0 && (now - key2_press_time >= LONG_PRESS_TIME))
        {
            key2_long_flag = 1;
            event = KEY2_LONG;
        }
    }
    else if (key2_cur == 1 && key2_last == 0)   // 释放
    {
        if (key2_long_flag == 0 && (now - key2_press_time >= SHORT_PRESS_MIN))
        {
            // 如果已经有按键1的事件，不覆盖（按键1优先）
            if (event == KEY_NONE)
                event = KEY2_SHORT;
        }
    }
    
    key1_last = key1_cur;
    key2_last = key2_cur;
    
    return event;
}
