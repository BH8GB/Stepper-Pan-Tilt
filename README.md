# 二轴步进电机云台控制系统

基于 STM32H743VIT6 + 2× A4988 + 2× MT6701QT 的串口控制云台系统

---

## 1. 系统概述

| 项目 | 参数 |
|------|------|
| 主控 | STM32H743VIT6 (LQFP100, 480MHz) |
| 驱动 | 2× A4988, 16 微步细分 |
| 编码器 | 2× MT6701QT (I2C, 14-bit) |
| 水平轴 | 360° 连续旋转, 减速比 15:90 (6:1) |
| 俯仰轴 | ±45°, 减速比 10:32 (3.2:1) |
| 控制 | USART1, 115200-8-N-1, ASCII 命令 |
| RTOS | FreeRTOS CMSIS_V2 |

---

## 2. GPIO 引脚分配表

### 2.1 编码器 I2C1

| 功能 | 引脚 | 说明 |
|------|------|------|
| I2C1_SCL | **PB6** | MT6701 时钟线 |
| I2C1_SDA | **PB7** | MT6701 数据线 |
| 编码器地址 | — | Pan: **0x06**, Tilt: **0x0E** |

### 2.2 串口 USART1

| 功能 | 引脚 | 说明 |
|------|------|------|
| USART1_TX | **PA9** | 串口发送 |
| USART1_RX | **PA10** | 串口接收 |

### 2.3 水平轴 (Pan) A4988

| 功能 | 引脚 | 方向 | 说明 |
|------|------|------|------|
| DIR | **PB0** | MCU→A4988 | 方向控制 |
| STEP | **PB1** | MCU→A4988 | 步进脉冲 (50kHz ISR) |
| EN | **PA0** | MCU→A4988 | 使能 (L=使能) |
| MS1 | **PA2** | MCU→A4988 | 微步选择 (H=16步) |
| MS2 | **PA3** | MCU→A4988 | 微步选择 (H=16步) |
| MS3 | **PA4** | MCU→A4988 | 微步选择 (H=16步) |
| SLEEP | **PA5** | MCU→A4988 | 休眠 (H=唤醒) |
| RESET | **PA6** | MCU→A4988 | 复位 (H=正常) |

### 2.4 俯仰轴 (Tilt) A4988

| 功能 | 引脚 | 方向 | 说明 |
|------|------|------|------|
| DIR | **PD0** | MCU→A4988 | 方向控制 |
| STEP | **PD1** | MCU→A4988 | 步进脉冲 (50kHz ISR) |
| EN | **PD2** | MCU→A4988 | 使能 (L=使能) |
| MS1 | **PD3** | MCU→A4988 | 微步选择 (H=16步) |
| MS2 | **PD4** | MCU→A4988 | 微步选择 (H=16步) |
| MS3 | **PD5** | MCU→A4988 | 微步选择 (H=16步) |
| SLEEP | **PD6** | MCU→A4988 | 休眠 (H=唤醒) |
| RESET | **PD7** | MCU→A4988 | 复位 (H=正常) |

---

## 3. 时钟树配置

```
HSI (64MHz)
  └─ PLL1: /4 × 60 / 1 = 480MHz → SYSCLK (CPU)
       ├─ HCLK = SYSCLK / 2 = 240MHz
       │    ├─ APB1 = HCLK / 2 = 120MHz → TIM7 时钟源 = 2×APB1 = 240MHz
       │    ├─ APB2 = HCLK / 2 = 120MHz → USART1 时钟源
       │    └─ I2C1 时钟 = 120MHz
       └─ QSPI 时钟 = HCLK = 240MHz

LSE (32.768kHz) → RTC
```

**关键时钟频率:**
- CPU (SYSCLK): **480MHz**
- HCLK: **240MHz**
- APB1 Timer Clock: **240MHz** (用于 TIM7)
- I2C1 Clock: **120MHz**
- USART1 Clock: **120MHz**

---

## 4. 定时器用法

| 定时器 | 用途 | 频率 | 配置 |
|--------|------|------|------|
| **TIM6** | FreeRTOS 时基 (HAL Tick) | 1kHz | PSC=119999, ARR=999 |
| **TIM7** | 步进脉冲 DDS 生成 (ISR) | **50kHz** | PSC=47, ARR=99 → 240M/48/100=50kHz |

**TIM7 中断优先级:** 抢占优先级=1 (最高实时性)
**USART1 中断优先级:** 抢占优先级=5
**FreeRTOS 任务优先级:** motorTask (High) > commandTask (Normal) > defaultTask (Low)

---

## 5. FreeRTOS 任务设计

| 任务 | 优先级 | 周期/方式 | 栈大小 | 功能 |
|------|--------|-----------|--------|------|
| **motorTask** | osPriorityHigh | **1ms** 周期 (vTaskDelayUntil) | 4KB | 编码器读取、位置误差计算、梯形速度规划、PID修正 |
| **commandTask** | osPriorityNormal | **事件驱动** (UART通知) | 2KB | 串口命令解析、执行、回复 |
| **defaultTask** | osPriorityLow | 500ms 周期 | 1KB | 心跳/监控 (可扩展 LED/看门狗) |

---

## 6. 运动控制算法

### 6.1 梯形加减速
- 参数: 最大速度 15000 step/s, 加速度 30000 step/s²
- 制导逻辑: 每 1ms 比较 "刹车距离" 与 "剩余距离"，自动切换加速/匀速/减速阶段
- 刹车距离: `s = v² / (2a)`

### 6.2 PID 位置闭环
- 参数: Kp=1.5, Ki=0.02, Kd=0.3 (可调整)
- 积分限幅: ±500 (防饱和)
- 死区: ±3 步 (约 ±0.056° Pan 或 ±0.105° Tilt)

### 6.3 DDS 步进脉冲生成
- ISR 频率 50kHz (20μs/周期)
- 脉冲宽度: 1 ISR 周期 (20μs) > A4988 最小 1μs
- 速度分辨率: 1 step/s

### 6.4 过零最短路径 (Pan 轴)
- 自动计算 360° 范围内最近路径
- 例: target=350°, current=10° → 转动 -20° 而非 +340°

---

## 7. 串口控制协议

**串口参数:** 115200 baud, 8 data bits, 1 stop bit, no parity

### 命令列表

| 命令 | 格式 | 示例 | 说明 |
|------|------|------|------|
| **PAN** | `PAN <角度>\n` | `PAN 90.5` | 水平轴转动到 90.5° |
| **TILT** | `TILT <角度>\n` | `TILT 20.0` | 俯仰轴转动到 +20° |
| **HOME** | `HOME\n` | `HOME` | 两轴回零 |
| **STOP** | `STOP\n` | `STOP` | 紧急停止 |
| **STATUS** | `STATUS\n` | `STATUS` | 查询两轴状态 |

### 响应示例

```
> PAN 90.5
OK: PAN -> 90.50

> TILT 50
ERROR: tilt out of range (50.00), valid range: -45 to 45

> STATUS
STATUS: Pan=90.50 deg (IDLE), Tilt=0.00 deg (IDLE)

> TILT -30
OK: TILT -> -30.00

> STATUS
STATUS: Pan=90.50 deg (IDLE), Tilt=-30.00 deg (MOVING)

> STOP
OK: STOPPED

> HOME
OK: HOME initiated
```

---

## 8. 硬件接线指南

### 8.1 MT6701 编码器接线

```
MT6701 (Pan)          STM32H743VIT6
  VCC  ────────────── 3.3V
  GND  ────────────── GND
  SCL  ────────────── PB6 (I2C1_SCL)
  SDA  ────────────── PB7 (I2C1_SDA)
  ADDR ────────────── GND  (地址 0x06)

MT6701 (Tilt)
  VCC  ────────────── 3.3V
  GND  ────────────── GND
  SCL  ────────────── PB6 (I2C1_SCL)  ← 共用总线
  SDA  ────────────── PB7 (I2C1_SDA)  ← 共用总线
  ADDR ────────────── 3.3V (地址 0x0E)
```

### 8.2 A4988 接线 (以 Pan 轴为例)

```
A4988 (Pan)           STM32H743VIT6     电机
  DIR   ───────────── PB0
  STEP  ───────────── PB1
  EN    ───────────── PA0
  MS1   ───────────── PA2
  MS2   ───────────── PA3
  MS3   ───────────── PA4
  SLEEP ───────────── PA5
  RESET ───────────── PA6
  VDD   ───────────── 3.3V
  VMOT  ───────────── 12V~24V (电机电源)
  GND   ───────────── GND (与 MCU 共地)
  1A/1B ───────────── 步进电机 A 相
  2A/2B ───────────── 步进电机 B 相
```

### 8.3 USART1 串口接线

```
USB-TTL 模块          STM32H743VIT6
  TXD  ────────────── PA10 (USART1_RX)
  RXD  ────────────── PA9  (USART1_TX)
  GND  ────────────── GND
```

---

## 9. Keil 工程编译说明

1. 用 STM32CubeMX 打开 `H743VIT6.ioc`，确认配置无误后 **Generate Code**
2. 在 Keil MDK-ARM 中打开 `MDK-ARM/H743VIT6/H743VIT6.uvprojx`
3. 将新增的源文件添加到工程:
   - 在 Project 窗口右键 → Add Existing Files
   - 添加 `Core/Src/mt6701.c`, `a4988.c`, `motor.c`, `protocol.c`
   - 添加 `Core/Inc/` 到 Include Paths
4. 编译 (F7)，下载 (F8)

---

## 10. CubeMX 配置要点 (.ioc)

在 CubeMX 中需确认以下配置:

| 外设 | 模式 | 关键参数 |
|------|------|----------|
| **I2C1** | I2C | Speed: 100kHz, PB6=SCL, PB7=SDA |
| **USART1** | Asynchronous | 115200-8-N-1, PA9=TX, PA10=RX |
| **TIM7** | Internal Clock | PSC=47, ARR=99, NVIC enabled |
| **FREERTOS** | CMSIS_V2 | 保持原有配置 |

---

## 11. 调试建议

1. **上电检查:** 上电后串口应输出欢迎信息和命令列表
2. **编码器验证:** 发送 `STATUS` 确认编码器读数正常 (0~360°)
3. **限位测试:** 发送 `TILT 50` 应返回 ERROR: tilt out of range
4. **运动测试:** 发送 `PAN 90` 观察水平轴是否平稳转动到 90°
5. **PID 调参:** 在 `motor.h` 中修改 MOTOR_DEFAULT_KP/KI/KD，调整响应速度和精度
6. **实时调试:** 可在 `motorTask` 中增加 printf 打印实时角度误差 (通过 SWV 或额外串口)
