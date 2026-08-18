# STM32 智能循迹小车（黑线循迹）

基于 **STM32F103** 的智能循迹小车，使用 STM32CubeMX 生成工程、Keil MDK 开发。

## 功能

- **PID 巡线**：红外循迹传感器阵列 + 位置式 PD 控制（`KP=85, KD=35`），实时纠偏
- **双电机差速**：PWM 调速，直行 / 左右转弯
- **路口 / 标记识别**：状态机（`APP_INIT → RUN_TO_B → ... → FINISH`），含消抖 + 冷却时间，避免误判
- **蜂鸣器提示**：起跑、到达标记点等状态提示

## 目录结构

```
.
├── Core/                      # 用户代码 + CubeMX 生成代码
│   ├── Inc/
│   │   ├── main.h
│   │   └── ...
│   └── Src/
│       ├── main.c             # 核心逻辑：巡线、PID、状态机、电机控制
│       └── ...
├── MDK-ARM/
│   ├── bisai.uvprojx          # Keil 工程文件
│   └── startup_stm32f103xb.s  # 启动文件
├── bisai.ioc                  # STM32CubeMX 配置文件
└── .mxproject
```

## 如何编译

> 本仓库未包含 ST 官方 HAL / CMSIS 库（`Drivers/` 目录），需要自行生成。

1. 用 **STM32CubeMX** 打开 `bisai.ioc`，点 `GENERATE CODE` 生成 `Drivers/` 库文件
2. 用 **Keil MDK** 打开 `MDK-ARM/bisai.uvprojx` 编译即可

## 硬件

- 主控：STM32F103（HSI 内部时钟）
- 驱动：双路电机（TB6612 类驱动，TIM2 PWM）
- 传感器：5 路红外循迹传感器阵列（黑线检测，低电平有效）
- 其他：蜂鸣器、I2C1（预留）

## 引脚配置

### 电机驱动

| 功能 | 引脚 | 说明 |
|---|---|---|
| STBY | PB11 | 驱动使能，高电平使能 |
| AIN1 | PB12 | 左电机方向 1 |
| AIN2 | PB13 | 左电机方向 2 |
| BIN1 | PB14 | 右电机方向 1 |
| BIN2 | PB15 | 右电机方向 2 |
| 左轮 PWM | PA0 | TIM2_CH1 |
| 右轮 PWM | PA1 | TIM2_CH2 |

### 循迹传感器（5 路红外，从左到右）

| 位置 | 引脚 | 说明 |
|---|---|---|
| 传感器 1（最左） | PA4 | 上拉输入 |
| 传感器 2 | PA5 | 上拉输入 |
| 传感器 3（中间） | PB0 | 上拉输入 |
| 传感器 4 | PB1 | 上拉输入 |
| 传感器 5（最右） | PB10 | 上拉输入 |

> 默认 `SENSOR_ACTIVE_LEVEL = GPIO_PIN_RESET`，即**低电平表示检测到黑线**。若你的传感器输出相反，改 `main.c` 顶部的 `SENSOR_ACTIVE_LEVEL` 为 `GPIO_PIN_SET`。

### 其他外设

| 功能 | 引脚 | 说明 |
|---|---|---|
| 蜂鸣器 | PA10 | 输出，高电平响 |
| I2C1_SCL | PB6 | 预留 |
| I2C1_SDA | PB7 | 预留 |

> 已通过 `__HAL_AFIO_REMAP_SWJ_NOJTAG()` 禁用 JTAG，仅保留 SWD 调试口（PA13/PA14）。

## 关键参数（`main.c` 顶部可调）

| 参数 | 值 | 说明 |
|---|---|---|
| `PID_KP` | 85 | 比例系数 |
| `PID_KD` | 35 | 微分系数 |
| `PWM_BASE_SPEED` | 620 | 直行基础速度 |
| `PWM_TURN_SPEED` | 430 | 转弯速度 |
| `MARKER_COUNT_MIN` | 4 | 路口判定传感器数量 |
