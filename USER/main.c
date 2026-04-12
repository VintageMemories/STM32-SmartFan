/**
 ******************************************************************************
 * @file    main.c
 * @author  城南旧梦
 * @version V1.0
 * @date    2026-03-28
 * @brief   基于STM32的智能温控风扇系统
 * 
 * 功能说明：
 *   1. 手动模式：5档风速可调（0%~100%），按键切换档位
 *   2. 自动模式：SHT30采集温度，增量式PID连续调节风扇转速
 *   3. 蓝牙模式：HC-05接收指令，支持远程切换模式和设置参数
 *   4. 状态显示：OLED实时显示温度、模式、档位/目标温度
 *   5. 掉电保存：备份寄存器保存配置，断电后自动恢复
 *   6. 连接指示：LED反映蓝牙状态（未连接闪烁，连接后熄灭）
 * 
 * 技术特点：
 *   - 控制算法：增量式PID，带死区处理、积分限幅、输出限幅
 *   - 通信方式：蓝牙串口，支持单字符/字符串两种指令格式
 *   - 显示优化：OLED局部刷新，避免闪烁
 *   - 架构设计：三模式状态机，模块化驱动分层
 * 
 * 引脚分配：
 *   | PA4  | PA6  | PA8  | PA9  | PA10 | PA11 | PB6  | PB7  | PB8  | PB9  | PC0  | PC1  |
 *   |------|------|------|------|------|------|------|------|------|------|------|------|
 *   | LED  | PWM  | STATE| TX   | RX   | EN   | OLED | OLED | SHT30| SHT30| GEAR | MODE |
 *   |      |      |      |      |      |      | SCL  | SDA  | SCL  | SDA  | KEY  | KEY  |
 * 
 * 按键说明：
 *   PC0短按：切换档位（0~4档）
 *   PC1短按：切换手动/自动模式
 *   PC1长按2秒：进入/退出蓝牙模式
 * 
 * 蓝牙指令：
 *   0~4(档位)  A/AUTO(自动)  M/MANUAL(手动)  X/E/EXIT(退出)
 *   TEMP:xx.x(目标温度，16.0~35.0℃)  GEAR:x(档位，0~4)
 * 
 * 配置参数（main.c开头）：
 *   DEFAULT_TARGET_TEMP：默认目标温度
 *   PID_KP/KI/KD：PID参数
 *   PID_DEAD_ZONE：死区范围
 *   LED_BLINK_PERIOD_MS：LED闪烁周期
 * 
 ******************************************************************************
 */

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "pwm.h"
#include "iic_oled.h"
#include "iic_sht30.h"
#include "led.h"
#include "exti.h"
#include "hc05.h"

/*============================================================================
 * 系统配置参数（修改这里的值即可调整系统行为）
 *============================================================================*/
#define DEFAULT_TARGET_TEMP     12.0f   // 开机默认目标温度（℃）
#define TEMP_MIN                16.0f   // 蓝牙可设最低温度
#define TEMP_MAX                35.0f   // 蓝牙可设最高温度
#define TEMP_ERROR             -99.0f   // 温度传感器读取失败时的返回值

// PID 控制参数（影响自动模式下的风扇响应速度）
#define PID_KP                  16.0f   // 比例系数：温差1℃ → 16% 输出
#define PID_KI                   0.2f   // 积分系数：消除稳态误差
#define PID_KD                   1.5f   // 微分系数：抑制超调和震荡
#define PID_OUTPUT_MAX         100.0f   // PID 输出上限（风扇最大占空比）
#define PID_OUTPUT_MIN           0.0f   // PID 输出下限（风扇最小占空比）
#define PID_INTEGRAL_MAX        30.0f   // 积分限幅：防止积分饱和过大
#define PID_DEAD_ZONE            0.1f   // 死区范围：温差在此范围内不调节风扇
#define PID_DIFF_OFF_THRESHOLD  -0.5f   // 温度低于目标超过此值，直接关闭风扇

// LED 闪烁参数（蓝牙模式未连接时使用）
#define LED_BLINK_PERIOD_MS     500     // 闪烁周期：完整一次亮灭需要500ms
#define LED_BLINK_HALF_MS       250     // 半周期：亮250ms，灭250ms

// 手动档位对应的 PWM 比较值（定时器 ARR=1000，比较值 = 占空比 × 10）
#define GEAR_PWM_0              0       // 0档：占空比 0%，风扇停转
#define GEAR_PWM_1              250     // 1档：占空比 25%
#define GEAR_PWM_2              500     // 2档：占空比 50%
#define GEAR_PWM_3              750     // 3档：占空比 75%
#define GEAR_PWM_4              1000    // 4档：占空比 100%

// 温度传感器故障时的安全保护值
#define ERROR_SAFE_GEAR         2       // 传感器故障时强制使用的档位
#define ERROR_SAFE_PID_OUTPUT   50.0f   // 传感器故障时 PID 输出的固定值

/*============================================================================
 * 全局变量
 *============================================================================*/
volatile uint32_t g_msTicks = 0;        // 系统毫秒计数器，SysTick 中断每 1ms 自增

// 档位 PWM 映射表：gearPWM[档位] = 定时器比较值
const uint16_t gearPWM[5] = {GEAR_PWM_0, GEAR_PWM_1, GEAR_PWM_2, GEAR_PWM_3, GEAR_PWM_4};

// PID 算法相关变量
static float pid_error = 0;             // 当前误差 = 当前温度 - 目标温度
static float pid_last_error = 0;        // 上一次的误差值，用于计算微分
static float pid_integral = 0;          // 误差的积分累加值
static float pid_derivative = 0;        // 误差的变化率（微分）
static float pid_output = 0;            // PID 最终输出值（0~100%）

uint8_t tem_error_flag = 0;             // 温度传感器状态：0=正常，1=读取失败

// 外部变量引用（定义在其他文件中）
extern u8 USART_RX_BUF[];               // 蓝牙串口接收缓冲区（usart.c）
extern u16 USART_RX_STA;                // 接收状态标志：bit15=接收完成，bit14=收到回车
extern BTWorkMode_t btWorkMode;         // 蓝牙模式下的子模式：手动/自动（hc05.c）

/*============================================================================
 * 风扇控制函数
 *============================================================================*/

/*
 * 设置风扇为指定档位（手动模式 / 蓝牙手动模式使用）
 * gear: 档位 0~4，0档停转，4档全速
 */
void Fan_SetGear(uint8_t gear)
{
    if (gear > 4) gear = 4;                 // 防止档位越界
    manualGear = gear;                      // 更新全局档位变量
    TIM_SetCompare1(TIM3, gearPWM[gear]);   // 设置 PWM 比较值
}

/*
 * 设置风扇连续占空比（自动模式 / 蓝牙自动模式使用）
 * duty: 占空比 0.0~100.0，超出范围自动限幅
 */
void Fan_SetDuty(float duty)
{
    if (duty < 0.0f) duty = 0.0f;          // 下限保护
    if (duty > 100.0f) duty = 100.0f;      // 上限保护
    TIM_SetCompare1(TIM3, (uint16_t)(duty * 10));  // ARR=1000，比较值 = 占空比 × 10
}

/*============================================================================
 * PID 连续调速算法
 *============================================================================*/

/*
 * 根据当前温度与目标温度的差值，计算风扇应输出的占空比
 * 调用后结果保存在 pid_output 变量中
 */
void PID_Auto_Adjust(void)
{
    float diff = currentTemp - targetTemp;      // 温差：正数=温度过高，负数=温度过低
    
    // 死区处理：温差在 ±0.1℃ 内不调节，避免风扇频繁启停
    if (diff > -PID_DEAD_ZONE && diff < PID_DEAD_ZONE)
        return;
    
    // 温度过低（低于目标超过0.5℃）：直接关闭风扇，清零积分
    if (diff < PID_DIFF_OFF_THRESHOLD)
    {
        pid_output = 0.0f;
        pid_integral = 0.0f;
    }
    else
    {
        pid_error = diff;
        pid_integral += pid_error;              // 积分累加
        
        // 积分限幅：防止积分项过大导致超调
        if (pid_integral > PID_INTEGRAL_MAX) pid_integral = PID_INTEGRAL_MAX;
        if (pid_integral < -PID_INTEGRAL_MAX) pid_integral = -PID_INTEGRAL_MAX;
        
        pid_derivative = pid_error - pid_last_error;  // 微分：误差变化率
        
        // PID 核心公式：输出 = Kp×误差 + Ki×积分 + Kd×微分
        pid_output = PID_KP * pid_error + PID_KI * pid_integral + PID_KD * pid_derivative;
        
        // 输出限幅
        if (pid_output > PID_OUTPUT_MAX) pid_output = PID_OUTPUT_MAX;
        if (pid_output < PID_OUTPUT_MIN) pid_output = PID_OUTPUT_MIN;
    }
    
    pid_last_error = pid_error;                 // 保存本次误差供下次使用
}

/*============================================================================
 * OLED 显示函数
 *============================================================================*/

// 清除 OLED 指定的一行（16x16字体，每行占2页，page=2清第2-3页）
static void OLED_ClearLine(uint8_t page)
{
    IIC_OLED_Set_Pos(0, page);
    for (uint8_t i = 0; i < 128; i++) IIC_OLED_WR_Byte(0, OLED_DATA);
    IIC_OLED_Set_Pos(0, page + 1);
    for (uint8_t i = 0; i < 128; i++) IIC_OLED_WR_Byte(0, OLED_DATA);
}

// 判断当前是否为自动模式（自动模式下不显示档位）
static uint8_t IsAutoMode(void)
{
    if (sysMode == MODE_AUTO) return 1;
    if (sysMode == MODE_BLUETOOTH && btWorkMode == WORK_MODE_AUTO) return 1;
    return 0;
}

/*
 * OLED 显示更新（局部刷新，避免闪烁）
 * 第一行：T:温度 C  G:档位
 * 第二行：M:模式  S:目标温度
 */
void OLED_Update(void)
{
    // 静态变量：记录上次显示的内容，只有变化时才刷新
    static float lastTemp = -99.0f;
    static uint8_t lastGear = 255;
    static SystemMode_t lastMode = 255;
    static BTWorkMode_t lastBTMode = 255;
    static float lastTarget = -99.0f;
    static uint8_t lastErrorFlag = 0;
    
    char buf[24];
    uint8_t isAuto = IsAutoMode();
    
    //========== 第一行：温度 + 档位 ==========
    // 温度变化、档位变化、故障状态变化、模式变化、蓝牙子模式变化时刷新
    if (currentTemp != lastTemp || manualGear != lastGear || 
        tem_error_flag != lastErrorFlag || sysMode != lastMode || 
        btWorkMode != lastBTMode)
    {
        OLED_ClearLine(2);      // 清除第2-3页
        
        // 根据故障状态和模式决定显示内容
        if (tem_error_flag)
            sprintf(buf, isAuto ? "T:ERROR" : "T:ERROR  G:%d", manualGear);
        else
            sprintf(buf, isAuto ? "T:%.1f C" : "T:%.1f C  G:%d", currentTemp, manualGear);
        
        IIC_OLED_Show_Str(0, 2, buf, 16);
        
        // 更新记录
        lastTemp = currentTemp;
        lastGear = manualGear;
        lastErrorFlag = tem_error_flag;
        lastBTMode = btWorkMode;
    }
    
    //========== 第二行：模式 + 目标温度 ==========
    // 系统模式变化、目标温度变化、蓝牙子模式变化时刷新
    if (sysMode != lastMode || targetTemp != lastTarget || btWorkMode != lastBTMode)
    {
        OLED_ClearLine(4);      // 清除第4-5页
        
        switch (sysMode)
        {
            case MODE_AUTO:
                sprintf(buf, "M:auto  S:%.1f", targetTemp);
                break;
            case MODE_MANUAL:
                sprintf(buf, "M:manual");
                break;
            case MODE_BLUETOOTH:
                if (btWorkMode == WORK_MODE_AUTO)
                    sprintf(buf, "M:BT_auto S:%.1f", targetTemp);
                else
                    sprintf(buf, "M:BT_manual");
                break;
        }
        IIC_OLED_Show_Str(0, 4, buf, 16);
        
        // 更新记录
        lastMode = sysMode;
        lastTarget = targetTemp;
        lastBTMode = btWorkMode;
    }
}

/*============================================================================
 * 系统控制函数
 *============================================================================*/

// 根据当前系统模式和工作模式控制风扇
static void Fan_Control(void)
{
    if (sysMode == MODE_BLUETOOTH)
    {
        // 蓝牙模式：根据 btWorkMode 决定控制方式
        if (btWorkMode == WORK_MODE_AUTO && !tem_error_flag)
        {
            PID_Auto_Adjust();          // 计算 PID 输出
            Fan_SetDuty(pid_output);    // 连续调速
        }
        else
        {
            Fan_SetGear(manualGear);    // 固定档位
        }
    }
    else if (sysMode == MODE_AUTO && !tem_error_flag)
    {
        // 普通自动模式
        PID_Auto_Adjust();
        Fan_SetDuty(pid_output);
    }
    else  // MODE_MANUAL
    {
        // 普通手动模式
        Fan_SetGear(manualGear);
    }
}

// LED 状态控制：仅在蓝牙模式且未连接时闪烁，其他情况熄灭
static void LED_Control(void)
{
    if (sysMode == MODE_BLUETOOTH && !HC05_IS_CONNECTED())
    {
        // 蓝牙模式未连接：用系统时间取模实现周期性闪烁
        uint32_t time_in_cycle = g_msTicks % LED_BLINK_PERIOD_MS;
        if (time_in_cycle < LED_BLINK_HALF_MS)
            LED_On();   // 前半周期亮
        else
            LED_Off();  // 后半周期灭
    }
    else
    {
        LED_Off();      // 其他情况一律熄灭
    }
}

// 处理蓝牙串口接收到的指令
static void Process_SerialCommands(void)
{
    extern volatile uint8_t rx_flag;
    extern volatile uint8_t rx_byte;
    
    // 处理字符串指令（如 "AUTO"、"TEMP:26.0"、"GEAR:3" 等）
    if (USART_RX_STA & 0x8000)          // bit15=1 表示接收完成
    {
        HC05_ParseCommand(USART_RX_BUF);
        USART_RX_STA = 0;               // 清零状态，准备下一次接收
    }
    
    // 处理单字符指令（如 'A'、'M'、'0'~'4' 等）
    if (rx_flag)
    {
        rx_flag = 0;
        uint8_t single_char_buf[4] = {rx_byte, '\r', '\n', 0};
        HC05_ParseCommand(single_char_buf);
    }
}

// 读取温度传感器数据
static void Read_Temperature(void)
{
    float temp = SHT30_ReadTemperature();
    
    if (temp > TEMP_ERROR && temp < 100.0f)     // 读取成功
    {
        currentTemp = temp;
        tem_error_flag = 0;
    }
    else                                        // 读取失败
    {
        tem_error_flag = 1;
        manualGear = ERROR_SAFE_GEAR;           // 使用安全档位
        pid_output = ERROR_SAFE_PID_OUTPUT;     // 使用安全输出值
    }
}

/*============================================================================
 * 主函数
 *============================================================================*/
int main(void)
{
    // 系统初始化
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Config(SystemCoreClock / 1000);     // 配置 SysTick 1ms 中断
    
    // 外设初始化
    uart_init(38400);           // 蓝牙串口
    PWM_Init();                 // 风扇 PWM
    EXTI_KEY_Init();            // 按键外部中断
    IIC_OLED_Init();            // OLED 显示屏
    LED_Init();                 // LED 指示灯
    SHT30_Init();               // 温度传感器
    HC05_Init();                // 蓝牙模块
    LoadConfig();               // 加载保存的配置
    
    // 开机默认状态：手动模式，0档，默认目标温度
    sysMode = MODE_MANUAL;
    manualGear = 0;
    targetTemp = DEFAULT_TARGET_TEMP;
    
    // 开机画面
    IIC_OLED_Clear();
    IIC_OLED_Show_Str(16, 2, "System Start", 16);
    delay_ms(1000);
    IIC_OLED_Clear();
    
    Fan_SetGear(manualGear);    // 风扇停转
    LED_SetMode(LED_MODE_ON);   // LED 初始状态
    
    // 主循环
    while (1)
    {
        Read_Temperature();             // 读取温度
        HC05_CheckConnection();         // 检测蓝牙连接状态变化
        Mode_Key_Process();             // 处理按键（短按/长按）
        Fan_Control();                  // 根据模式控制风扇
        OLED_Update();                  // 更新 OLED 显示
        LED_Control();                  // 更新 LED 状态
        Process_SerialCommands();       // 处理蓝牙串口指令
    }
}

/*============================================================================
 * SysTick 中断服务函数（1ms）
 *============================================================================*/
void SysTick_Handler(void)
{
    g_msTicks++;    // 系统毫秒计数器自增
}