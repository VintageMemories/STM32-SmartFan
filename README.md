# 基于STM32的智能温控风扇系统

基于 STM32F103RC 的 PID 温控风扇系统，支持手动/自动/蓝牙三种模式，OLED 实时显示，掉电保存配置。

## 项目简介

本项目基于 STM32F103RC 构建，通过 SHT30 采集环境温度，增量式 PID 算法连续调节风扇转速。支持手动档位、自动温控、蓝牙遥控三种工作模式，OLED 实时显示状态，备份寄存器实现配置掉电保存。独立看门狗防止程序卡死。

## 功能特点

- **三模式切换**：手动模式固定档位输出（0~4档），自动模式 PID 连续调速，蓝牙模式支持远程控制及子模式切换，按键短按/长按实现模式切换
- **PID 控制算法**：增量式 PID，带死区处理（±0.1℃）避免频繁启停，积分限幅防止饱和，温差<-0.5℃时自动关闭风扇
- **蓝牙通信**：HC-05 串口透传，支持单字符（0~4/A/M/X）和字符串（AUTO/MANUAL/TEMP:/GEAR:）指令
- **OLED 显示**：局部刷新，仅变化时更新对应行，避免闪烁
- **掉电保存**：备份寄存器保存目标温度、系统模式、手动档位，断电后自动恢复
- **看门狗保护**：独立看门狗 1 秒超时，程序卡死自动复位

## 工程架构

驱动层与业务层完全解耦，便于移植：

- 业务层
  - `USER/main.c`：系统状态管理、模式切换、蓝牙指令执行、按键业务、配置保存
  - `USER/config.h`：所有可调参数集中管理（PID、温度阈值、LED周期等）
- 驱动层
  - `HARDWARE/HC05/`：HC-05 蓝牙模块驱动
  - `HARDWARE/PID/`：增量式 PID 算法模块
  - `HARDWARE/SHT30/`：SHT30 温湿度传感器驱动
  - `HARDWARE/OLED/`：OLED 显示驱动
  - `HARDWARE/LED/`：LED 指示灯驱动
  - `HARDWARE/KEY/`：按键扫描驱动
- 系统层
  - `SYSTEM/`：延时、系统初始化、串口、PWM、外部中断
- 库文件
  - `STM32F10x_FWLib/`：STM32 标准外设库
  - `CORE/`：ARM Cortex-M3 内核文件

## 快速开始

1. 使用 Keil MDK 或 GCC Makefile 编译工程
2. 烧录至 STM32F103RC 开发板
3. 上电后系统进入手动模式，OLED 显示开机画面
4. PC1 短按切换手动/自动模式，长按 2 秒进入蓝牙模式
5. 手机蓝牙连接 HC-05（38400 波特率），发送指令控制

## 蓝牙指令

| 指令 | 功能 | 示例 |
|------|------|------|
| `0`~`4` | 设置档位 | `2` |
| `A` / `AUTO` | 切换到自动模式 | `A` |
| `M` / `MANUAL` | 切换到手动模式 | `M` |
| `TEMP:xx.x` | 设置目标温度（16.0~35.0℃） | `TEMP:26.0` |
| `GEAR:x` | 设置档位（0~4） | `GEAR:3` |
| `X` / `E` / `EXIT` | 退出蓝牙模式 | `X` |

## 引脚分配

| 引脚 | 功能 | 引脚 | 功能 |
|------|------|------|------|
| PA4 | LED | PB6 | OLED SCL |
| PA6 | 风扇 PWM | PB7 | OLED SDA |
| PA8 | BT STATE | PB8 | SHT30 SCL |
| PA9 | BT TX | PB9 | SHT30 SDA |
| PA10 | BT RX | PC0 | 档位按键 |
| PA11 | BT EN | PC1 | 模式按键 |

## 目录结构

```
基于STM32的智能温控风扇系统/
├── USER/
│   ├── main.c
│   ├── config.h
│   └── common.h
├── HARDWARE/
│   ├── HC05/
│   │   ├── hc05.h
│   │   └── hc05.c
│   ├── PID/
│   │   ├── pid.h
│   │   └── pid.c
│   ├── SHT30/
│   │   ├── iic_sht30.h
│   │   └── iic_sht30.c
│   ├── OLED/
│   │   ├── iic_oled.h
│   │   └── iic_oled.c
│   ├── LED/
│   │   ├── led.h
│   │   └── led.c
│   └── KEY/
│       ├── key.h
│       └── key.c
├── SYSTEM/
│   ├── delay/
│   ├── sys/
│   ├── usart/
│   ├── exti/
│   └── pwm/
├── STM32F10x_FWLib/
├── CORE/
├── Makefile
├── startup_stm32f103xe.s
└── STM32F103RCTx_FLASH.ld
```

## 配置参数

所有可调参数集中在 `USER/config.h`：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| DEFAULT_TARGET_TEMP | 12.0℃ | 开机默认目标温度 |
| PID_KP | 16.0 | 比例系数 |
| PID_KI | 0.2 | 积分系数 |
| PID_KD | 1.5 | 微分系数 |
| PID_DEAD_ZONE | 0.1℃ | 死区范围 |
| LED_BLINK_PERIOD_MS | 500ms | LED 闪烁周期 |

## 版权说明

本项目仅用于学习与展示，保留所有权利。
