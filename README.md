# stm32-line-following-car

比赛用的循迹小车，STM32F103 加 5 路红外传感器，跑黑线。CubeMX 生成工程，Keil 编译。

## 大概做了什么

- 巡线用的是位置式 PD，KP 和 KD 在 main.c 顶部，现场调过几轮
- 两个电机 PWM 差速转弯，直行、左右转
- 跑的过程中要过两个标记点（B、C），用状态机切，到点会停一下、响三声蜂鸣器再走
- 上电先响一声，跑到终点就停了

## 目录

```
.
├── Core/                      # 代码基本都在这，main.c 是核心
│   ├── Inc/
│   └── Src/
│       └── main.c             # 巡线、PID、状态机、电机控制
├── MDK-ARM/
│   ├── bisai.uvprojx          # Keil 工程
│   └── startup_stm32f103xb.s
├── bisai.ioc                  # CubeMX 配置
└── .mxproject
```

## 怎么编译

Drivers 没传（ST 的 HAL 库太大了）。拿到手先用 CubeMX 打开 `bisai.ioc`，重新 Generate 一下生成 Drivers，再用 Keil 打开 `MDK-ARM/bisai.uvprojx` 编译。

## 引脚

### 电机

| 功能 | 引脚 | 备注 |
|---|---|---|
| STBY | PB11 | 使能，拉高才转 |
| AIN1 | PB12 | 左电机方向 |
| AIN2 | PB13 | 左电机方向 |
| BIN1 | PB14 | 右电机方向 |
| BIN2 | PB15 | 右电机方向 |
| 左轮 PWM | PA0 | TIM2_CH1 |
| 右轮 PWM | PA1 | TIM2_CH2 |

### 循迹（5 路，从左到右）

| 位置 | 引脚 |
|---|---|
| 1（最左） | PA4 |
| 2 | PA5 |
| 3（中间） | PB0 |
| 4 | PB1 |
| 5（最右） | PB10 |

> 默认黑线是低电平（`SENSOR_ACTIVE_LEVEL = GPIO_PIN_RESET`）。如果传感器反过来，把 main.c 顶部这个宏改成 `GPIO_PIN_SET` 就行。

### 其他

| 功能 | 引脚 |
|---|---|
| 蜂鸣器 | PA10 |
| I2C1_SCL | PB6 |
| I2C1_SDA | PB7 |

> JTAG 关了，只留了 SWD（PA13/PA14）。

## 几个能调的参数

都在 main.c 最上面，现场根据场地调：

| 参数 | 值 | 干嘛的 |
|---|---|---|
| PID_KP | 85 | 比例 |
| PID_KD | 35 | 微分 |
| PWM_BASE_SPEED | 620 | 直行速度 |
| PWM_TURN_SPEED | 430 | 转弯速度 |
| MARKER_COUNT_MIN | 4 | 几个传感器同时压线算路口 |
