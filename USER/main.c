/**
 ******************************************************************************
 * @file    main.c
 * @author  城南旧梦
 * @version V2.0
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
 * 配置文件（config.h）：
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
#include "key.h"
#include "hc05.h"
#include "pid.h"
#include "config.h"

/*============================================================================
 * 系统模式定义
 *============================================================================*/
typedef enum {
    MODE_AUTO = 0,          /* 自动模式 */
    MODE_MANUAL,            /* 手动模式 */
    MODE_BLUETOOTH          /* 蓝牙模式 */
} SystemMode_t;

typedef enum {
    WORK_MODE_MANUAL = 0,   /* 蓝牙手动 */
    WORK_MODE_AUTO          /* 蓝牙自动 */
} BTWorkMode_t;

/*============================================================================
 * 全局变量
 *============================================================================*/
volatile uint32_t g_msTicks  = 0;                    /* 系统毫秒计数器，SysTick中断每1ms自增 */

float            currentTemp = 25.0f;                /* 当前温度(℃) */
float            targetTemp  = DEFAULT_TARGET_TEMP;  /* 目标温度(℃) */
uint8_t          manualGear  = 0;                    /* 手动档位(0~4) */
uint8_t          btConnected = 0;                    /* 蓝牙连接状态：1=已连接 */
SystemMode_t     sysMode     = MODE_MANUAL;          /* 系统当前工作模式 */
BTWorkMode_t     btWorkMode  = WORK_MODE_MANUAL;     /* 蓝牙模式下的子模式 */

/* 档位PWM比较值映射表：gearPWM[档位] = 定时器比较值 */
const uint16_t gearPWM[5] = {GEAR_PWM_0, GEAR_PWM_1, GEAR_PWM_2, GEAR_PWM_3, GEAR_PWM_4};

static PID_Controller g_pid;            /* PID控制器实例 */
static uint8_t tem_error_flag  = 0;     /* 温度传感器状态：0=正常，1=读取失败 */
static uint8_t fan_off_by_temp = 0;     /* 因温度过低而关闭风扇的标志 */

/* 外部变量引用 */
extern u8  USART_RX_BUF[];              /* 蓝牙串口接收缓冲区(usart.c) */
extern u16 USART_RX_STA;                /* 接收状态标志：bit15=接收完成 */

/*============================================================================
 * 函数声明
 *============================================================================*/
static void PID_Config_Init(void);
static void Read_Temperature(void);
static void Temp_Auto_Control(void);
static void Fan_Control(void);
static void LED_Control(void);
static void OLED_Update(void);
static void Process_SerialCommands(void);
static void Process_BluetoothCmd(HC05_Result_t result);
static void Key_Process(void);
static void SaveConfig(void);
static void LoadConfig(void);

/*============================================================================
 * 风扇控制函数
 *============================================================================*/

/**
 * @brief  设置风扇为指定档位（手动模式 / 蓝牙手动模式使用）
 * @param  gear  档位 0~4，0档停转，4档全速
 */
void Fan_SetGear(uint8_t gear)
{
    if (gear > 4) gear = 4;                 /* 防止档位越界 */
    manualGear = gear;                      /* 更新全局档位变量 */
    TIM_SetCompare1(TIM3, gearPWM[gear]);   /* 设置PWM比较值 */
}

/**
 * @brief  设置风扇连续占空比（自动模式 / 蓝牙自动模式使用）
 * @param  duty  占空比 0.0~100.0，超出范围自动限幅
 */
void Fan_SetDuty(float duty)
{
    if (duty < 0.0f)   duty = 0.0f;        /* 下限保护 */
    if (duty > 100.0f) duty = 100.0f;      /* 上限保护 */
    TIM_SetCompare1(TIM3, (uint16_t)(duty * 10));  /* ARR=1000，比较值=占空比×10 */
}

/*============================================================================
 * PID初始化
 *============================================================================*/

/**
 * @brief  初始化PID控制器参数
 *         参数值在config.h中定义，修改config.h即可调整PID行为
 */
static void PID_Config_Init(void)
{
    PID_Init(&g_pid, PID_KP, PID_KI, PID_KD,
             PID_OUTPUT_MAX, PID_OUTPUT_MIN,
             PID_INTEGRAL_MAX, PID_DEAD_ZONE);
    PID_SetSetpoint(&g_pid, targetTemp);
}

/*============================================================================
 * 温度读取
 *============================================================================*/

/**
 * @brief  读取SHT30温度传感器数据
 *         读取失败时进入故障保护：使用安全档位和固定占空比
 */
static void Read_Temperature(void)
{
    float temp = SHT30_ReadTemperature();
    
    if (temp > TEMP_ERROR && temp < 100.0f)
    {
        /* 读取成功 */
        currentTemp     = temp;
        tem_error_flag  = 0;
    }
    else
    {
        /* 读取失败，进入故障保护状态 */
        tem_error_flag  = 1;
        manualGear      = ERROR_SAFE_GEAR;
        PID_Reset(&g_pid);
    }
}

/*============================================================================
 * 自动模式温度调节
 *============================================================================*/

/**
 * @brief  自动模式温度调节（业务逻辑 + PID计算）
 *         温度低于目标超过阈值 → 直接关闭风扇并重置PID
 *         温度回升到恢复阈值 → 重新启用PID调节
 *         正常情况 → PID连续调节风扇转速
 */
static void Temp_Auto_Control(void)
{
    if (tem_error_flag) return;             /* 传感器故障不调节 */
    
    float diff = currentTemp - targetTemp;  /* 温差：正数=温度过高，负数=温度过低 */
    
    /* 温度过低（低于目标超过0.5℃）：直接关闭风扇，清零积分 */
    if (diff < TEMP_DIFF_OFF_THRESHOLD)
    {
        Fan_SetDuty(0.0f);
        PID_Reset(&g_pid);
        PID_SetSetpoint(&g_pid, targetTemp);
        fan_off_by_temp = 1;
        return;
    }
    
    /* 温度回升：恢复PID调节 */
    if (fan_off_by_temp && diff >= TEMP_DIFF_RESUME)
    {
        fan_off_by_temp = 0;
        PID_Reset(&g_pid);
        PID_SetSetpoint(&g_pid, targetTemp);
    }
    
    /* 关闭标记未清除，继续保持风扇关闭 */
    if (fan_off_by_temp)
    {
        Fan_SetDuty(0.0f);
        return;
    }
    
    /* 正常PID调节：将PID输出值转换为PWM占空比 */
    Fan_SetDuty(PID_Update(&g_pid, currentTemp));
}

/*============================================================================
 * 风扇总控
 *============================================================================*/

/**
 * @brief  根据当前系统模式和蓝牙子模式控制风扇
 *         手动模式 → 固定档位
 *         自动模式 → PID连续调速
 *         蓝牙模式 → 根据btWorkMode决定手动或自动
 */
static void Fan_Control(void)
{
    if (sysMode == MODE_BLUETOOTH && btWorkMode == WORK_MODE_AUTO)
        Temp_Auto_Control();                /* 蓝牙自动模式 */
    else if (sysMode == MODE_AUTO)
        Temp_Auto_Control();                /* 普通自动模式 */
    else
        Fan_SetGear(manualGear);            /* 手动模式 / 蓝牙手动模式 */
}

/*============================================================================
 * LED状态控制
 *============================================================================*/

/**
 * @brief  LED状态控制：仅在蓝牙模式且未连接时闪烁，其他情况熄灭
 */
static void LED_Control(void)
{
    if (sysMode == MODE_BLUETOOTH && !btConnected)
    {
        /* 蓝牙模式未连接：用系统时间取模实现周期性闪烁 */
        if ((g_msTicks % LED_BLINK_PERIOD_MS) < LED_BLINK_HALF_MS)
            LED_On();       /* 前半周期亮 */
        else
            LED_Off();      /* 后半周期灭 */
    }
    else
    {
        LED_Off();          /* 其他情况一律熄灭 */
    }
}

/*============================================================================
 * OLED显示更新
 *============================================================================*/

/**
 * @brief  判断当前是否为自动模式（自动模式下不显示档位）
 * @return 1=自动模式，0=非自动模式
 */
static uint8_t IsAutoMode(void)
{
    if (sysMode == MODE_AUTO) return 1;
    if (sysMode == MODE_BLUETOOTH && btWorkMode == WORK_MODE_AUTO) return 1;
    return 0;
}

/**
 * @brief  OLED显示更新（局部刷新，避免闪烁）
 *         第一行：T:温度 C  G:档位（自动模式不显示档位）
 *         第二行：M:模式  S:目标温度（手动模式不显示目标温度）
 */
static void OLED_Update(void)
{
    /* 静态变量：记录上次显示的内容，只有变化时才刷新 */
    static float        lastTemp      = -99.0f;
    static uint8_t      lastGear      = 255;
    static SystemMode_t lastMode      = 255;
    static BTWorkMode_t lastBTMode    = 255;
    static float        lastTarget    = -99.0f;
    static uint8_t      lastErrorFlag = 0;
    
    char buf[24];
    uint8_t isAuto = IsAutoMode();
    
    /* 第一行：温度 + 档位 */
    if (currentTemp != lastTemp || manualGear != lastGear || 
        tem_error_flag != lastErrorFlag || sysMode != lastMode || 
        btWorkMode != lastBTMode)
    {
        /* 清除第2-3页 */
        IIC_OLED_Set_Pos(0, 2);
        for (uint8_t i = 0; i < 128; i++) IIC_OLED_WR_Byte(0, OLED_DATA);
        IIC_OLED_Set_Pos(0, 3);
        for (uint8_t i = 0; i < 128; i++) IIC_OLED_WR_Byte(0, OLED_DATA);
        
        /* 根据故障状态和模式决定显示内容 */
        if (tem_error_flag)
            sprintf(buf, isAuto ? "T:ERROR" : "T:ERROR  G:%d", manualGear);
        else
            sprintf(buf, isAuto ? "T:%.1f C" : "T:%.1f C  G:%d", currentTemp, manualGear);
        
        IIC_OLED_Show_Str(0, 2, buf, 16);
        
        /* 更新记录 */
        lastTemp      = currentTemp;
        lastGear      = manualGear;
        lastErrorFlag = tem_error_flag;
        lastBTMode    = btWorkMode;
    }
    
    /* 第二行：模式 + 目标温度 */
    if (sysMode != lastMode || targetTemp != lastTarget || btWorkMode != lastBTMode)
    {
        /* 清除第4-5页 */
        IIC_OLED_Set_Pos(0, 4);
        for (uint8_t i = 0; i < 128; i++) IIC_OLED_WR_Byte(0, OLED_DATA);
        IIC_OLED_Set_Pos(0, 5);
        for (uint8_t i = 0; i < 128; i++) IIC_OLED_WR_Byte(0, OLED_DATA);
        
        /* 根据系统模式显示不同内容 */
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
        
        /* 更新记录 */
        lastMode   = sysMode;
        lastTarget = targetTemp;
        lastBTMode = btWorkMode;
    }
}

/*============================================================================
 * 蓝牙指令处理（业务逻辑）
 *============================================================================*/

/**
 * @brief  处理蓝牙解析后的指令，执行对应的业务操作
 * @param  result  蓝牙指令解析结果
 */
static void Process_BluetoothCmd(HC05_Result_t result)
{
    switch (result.cmd)
    {
        case HC05_CMD_GEAR_0:
        case HC05_CMD_GEAR_1:
        case HC05_CMD_GEAR_2:
        case HC05_CMD_GEAR_3:
        case HC05_CMD_GEAR_4:
            /* 档位指令：设置档位并保存 */
            manualGear = (uint8_t)result.value;
            if (sysMode != MODE_AUTO && 
                !(sysMode == MODE_BLUETOOTH && btWorkMode == WORK_MODE_AUTO))
                Fan_SetGear(manualGear);
            SaveConfig();
            {
                char ack[16];
                sprintf(ack, "Gear:%d\r\n", manualGear);
                HC05_SendString(ack);
            }
            break;
        
        case HC05_CMD_AUTO:
            /* 自动模式 */
            if (sysMode == MODE_BLUETOOTH) btWorkMode = WORK_MODE_AUTO;
            else sysMode = MODE_AUTO;
            PID_Reset(&g_pid);
            PID_SetSetpoint(&g_pid, targetTemp);
            HC05_SendString("Auto\r\n");
            break;
        
        case HC05_CMD_MANUAL:
            /* 手动模式 */
            if (sysMode == MODE_BLUETOOTH) btWorkMode = WORK_MODE_MANUAL;
            else sysMode = MODE_MANUAL;
            HC05_SendString("Manual\r\n");
            break;
        
        case HC05_CMD_EXIT:
            /* 退出蓝牙 */
            sysMode    = MODE_MANUAL;
            btWorkMode = WORK_MODE_MANUAL;
            HC05_SendString("Exit\r\n");
            break;
        
        case HC05_CMD_SET_TEMP:
            /* 设置目标温度 */
            if (result.value >= TEMP_MIN && result.value <= TEMP_MAX)
            {
                targetTemp = result.value;
                PID_SetSetpoint(&g_pid, targetTemp);
                SaveConfig();
                {
                    char ack[32];
                    sprintf(ack, "Target:%.1f\r\n", targetTemp);
                    HC05_SendString(ack);
                }
            }
            break;
        
        case HC05_CMD_SET_GEAR:
            /* 设置档位 */
        {
            uint8_t g = (uint8_t)result.value;
            if (g <= 4)
            {
                manualGear = g;
                if (sysMode != MODE_AUTO && 
                    !(sysMode == MODE_BLUETOOTH && btWorkMode == WORK_MODE_AUTO))
                    Fan_SetGear(manualGear);
                SaveConfig();
                {
                    char ack[16];
                    sprintf(ack, "Gear:%d\r\n", manualGear);
                    HC05_SendString(ack);
                }
            }
            break;
        }
        
        default:
            break;
    }
}

/**
 * @brief  处理蓝牙串口指令（从usart缓冲区读取并解析）
 */
static void Process_SerialCommands(void)
{
    extern volatile uint8_t rx_flag;
    extern volatile uint8_t rx_byte;
    
    /* 字符串指令处理（如"AUTO"、"TEMP:26.0"、"GEAR:3"等） */
    if (USART_RX_STA & 0x8000)              /* bit15=1表示接收完成 */
    {
        HC05_Result_t result = HC05_ParseCommand(USART_RX_BUF);
        if (result.cmd != HC05_CMD_NONE)
            Process_BluetoothCmd(result);
        USART_RX_STA = 0;                   /* 清零状态，准备下一次接收 */
    }
    
    /* 单字符指令处理（如'A'、'M'、'0'~'4'等） */
    if (rx_flag)
    {
        rx_flag = 0;
        uint8_t buf[4] = {rx_byte, '\r', '\n', 0};
        HC05_Result_t result = HC05_ParseCommand(buf);
        if (result.cmd != HC05_CMD_NONE)
            Process_BluetoothCmd(result);
    }
}

/*============================================================================
 * 蓝牙连接检测
 *============================================================================*/

/**
 * @brief  检测蓝牙连接状态变化并更新LED
 */
static void BT_Connection_Check(void)
{
    static uint8_t lastState = 0;
    uint8_t curState = HC05_IS_CONNECTED();
    
    if (curState != lastState)
    {
        lastState   = curState;
        btConnected = curState;
        
        if (sysMode == MODE_BLUETOOTH)
        {
            if (btConnected)
                LED_SetMode(LED_MODE_ON);
            else
                LED_SetMode(LED_MODE_FAST);
        }
    }
}

/*============================================================================
 * 按键处理（业务逻辑，调用key.c的KEY_Scan获取事件）
 *============================================================================*/

/**
 * @brief  按键业务处理（主循环调用）
 *         PC0(KEY1)短按：切换档位
 *         PC1(KEY2)短按：切换手动/自动模式
 *         PC1(KEY2)长按：进入/退出蓝牙模式
 */
static void Key_Process(void)
{
    Key_Event_t event = KEY_Scan();
    
    switch (event)
    {
        case KEY1_SHORT:
            /* PC0短按：手动模式/蓝牙手动模式下切换档位 */
            if (sysMode == MODE_MANUAL || 
                (sysMode == MODE_BLUETOOTH && btWorkMode == WORK_MODE_MANUAL))
            {
                manualGear = (manualGear + 1) % 5;
                Fan_SetGear(manualGear);
            }
            break;
        
        case KEY2_SHORT:
            /* PC1短按：非蓝牙模式下切换手动/自动 */
            if (sysMode != MODE_BLUETOOTH)
            {
                sysMode = (sysMode == MODE_MANUAL) ? MODE_AUTO : MODE_MANUAL;
                PID_Reset(&g_pid);
                PID_SetSetpoint(&g_pid, targetTemp);
            }
            break;
        
        case KEY2_LONG:
            /* PC1长按：进入/退出蓝牙模式 */
            if (sysMode == MODE_BLUETOOTH)
            {
                /* 退出蓝牙：恢复之前的状态 */
                sysMode = (btWorkMode == WORK_MODE_MANUAL) ? MODE_MANUAL : MODE_AUTO;
                LED_SetMode(LED_MODE_ON);
            }
            else
            {
                /* 进入蓝牙 */
                btWorkMode = (sysMode == MODE_MANUAL) ? WORK_MODE_MANUAL : WORK_MODE_AUTO;
                sysMode = MODE_BLUETOOTH;
                LED_SetMode(HC05_IS_CONNECTED() ? LED_MODE_ON : LED_MODE_FAST);
            }
            PID_Reset(&g_pid);
            PID_SetSetpoint(&g_pid, targetTemp);
            break;
        
        default:
            break;
    }
}

/*============================================================================
 * 掉电保存（备份寄存器）
 *============================================================================*/

/**
 * @brief  保存配置到备份寄存器
 */
static void SaveConfig(void)
{
    uint16_t tempInt = (uint16_t)(targetTemp * 10);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    
    BKP_WriteBackupRegister(BKP_DR1, tempInt);
    BKP_WriteBackupRegister(BKP_DR2, (uint16_t)sysMode);
    BKP_WriteBackupRegister(BKP_DR3, (uint16_t)manualGear);
}

/**
 * @brief  从备份寄存器加载配置
 */
static void LoadConfig(void)
{
    uint16_t tempInt;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    
    tempInt = BKP_ReadBackupRegister(BKP_DR1);
    if (tempInt >= 160 && tempInt <= 350)
    {
        targetTemp = tempInt / 10.0f;
        sysMode    = (SystemMode_t)BKP_ReadBackupRegister(BKP_DR2);
        manualGear = (uint8_t)BKP_ReadBackupRegister(BKP_DR3);
        
        if (manualGear > 4)              manualGear = 2;
        if (sysMode > MODE_BLUETOOTH)    sysMode = MODE_AUTO;
    }
}

/*============================================================================
 * 主函数
 *============================================================================*/
int main(void)
{
    /* 系统初始化 */
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Config(SystemCoreClock / 1000);     /* 配置SysTick 1ms中断 */
    
    /* 外设初始化 */
    uart_init(38400);           /* 蓝牙串口 */
    PWM_Init();                 /* 风扇PWM */
    EXTI_KEY_Init();            /* 按键外部中断（只初始化硬件） */
    KEY_Init();                 /* 按键GPIO初始化 */
    IIC_OLED_Init();            /* OLED显示屏 */
    LED_Init();                 /* LED指示灯 */
    SHT30_Init();               /* SHT30温度传感器 */
    HC05_Init();                /* HC-05蓝牙模块 */
    PID_Config_Init();          /* PID控制器 */
    LoadConfig();               /* 加载掉电保存的配置 */
    
    /* 开机默认状态 */
    PID_SetSetpoint(&g_pid, targetTemp);
    
    /* 开机画面 */
    IIC_OLED_Clear();
    IIC_OLED_Show_Str(16, 2, "System Start", 16);
    delay_ms(1000);
    IIC_OLED_Clear();
    
    Fan_SetGear(manualGear);
    LED_SetMode(LED_MODE_ON);
    
    /* 主循环 */
    while (1)
    {
        Read_Temperature();             /* 读取温度 */
        BT_Connection_Check();          /* 检测蓝牙连接状态变化 */
        Key_Process();                  /* 处理按键（调用key.c扫描） */
        Fan_Control();                  /* 根据模式控制风扇 */
        OLED_Update();                  /* 更新OLED显示 */
        LED_Control();                  /* 更新LED状态 */
        Process_SerialCommands();       /* 处理蓝牙串口指令 */
    }
}

/*============================================================================
 * SysTick中断服务函数（1ms）
 *============================================================================*/
void SysTick_Handler(void)
{
    g_msTicks++;    /* 系统毫秒计数器自增 */
}