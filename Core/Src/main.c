/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_INIT = 0,
  APP_RUN_TO_B,
  APP_STOP_AT_B,
  APP_RUN_TO_C,
  APP_STOP_AT_C,
  APP_FINISH
} AppState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_ACTIVE_LEVEL   GPIO_PIN_RESET   // 黑线输出低电平就用 RESET；如果相反改成 SET

#define PWM_MAX_VALUE         999
#define PWM_BASE_SPEED        620
#define PWM_TURN_SPEED        430
#define PWM_MIN_SPEED         320

#define PID_KP                85
#define PID_KD                35

#define MARKER_COUNT_MIN      4      // 4个及以上传感器同时检测到线，认为是路口/标记
#define MARKER_HOLD_MS        60     // 路口判定稳定时间
#define START_IGNORE_MS       1200    // 上电后前 1.2s 忽略标记，防止起跑就误判
#define MARKER_COOLDOWN_MS     700    // 识别一次标记后，冷却时间，防止重复触发
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static uint8_t  last_dir = 0;      // 0=直行, 1=右偏, 2=左偏
static int16_t  last_error = 0;

static AppState_t app_state = APP_INIT;
static uint32_t state_tick = 0;
static uint32_t marker_enter_tick = 0;
static uint32_t last_marker_trigger_tick = 0;
static uint8_t marker_latched = 0;

static uint8_t b_done = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Beep(uint16_t ms);
void Buzzer_Beep3(void);

void Motor_Enable(void);
void Motor_Set(uint16_t left_pwm, uint16_t right_pwm);
void Motor_Stop(void);

uint8_t LineSensor_Read(void);
uint8_t LineSensor_Count(uint8_t s);
int16_t  LineSensor_Error(uint8_t s, uint8_t *count);

void LineFollow_Task(void);
void App_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Buzzer_On(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
}

void Buzzer_Off(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

void Buzzer_Beep(uint16_t ms)
{
  Buzzer_On();
  HAL_Delay(ms);
  Buzzer_Off();
}

void Buzzer_Beep3(void)
{
  for (uint8_t i = 0; i < 3; i++)
  {
    Buzzer_Beep(120);
    HAL_Delay(120);
  }
}

void Motor_Enable(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // STBY = 1
}

static uint16_t clamp_pwm(int32_t v)
{
  if (v < 0) return 0;
  if (v > PWM_MAX_VALUE) return PWM_MAX_VALUE;
  return (uint16_t)v;
}

void Motor_Set(uint16_t left_pwm, uint16_t right_pwm)
{
  Motor_Enable();

  // 默认前进
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // AIN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // AIN2

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // BIN1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // BIN2

  if (left_pwm > PWM_MAX_VALUE)  left_pwm  = PWM_MAX_VALUE;
  if (right_pwm > PWM_MAX_VALUE) right_pwm = PWM_MAX_VALUE;

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, right_pwm);
}

void Motor_Stop(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

uint8_t LineSensor_Read(void)
{
  uint8_t s = 0;

  // bit0~bit4: 左到右五个传感器
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)  == SENSOR_ACTIVE_LEVEL) s |= (1 << 0);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)  == SENSOR_ACTIVE_LEVEL) s |= (1 << 1);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)  == SENSOR_ACTIVE_LEVEL) s |= (1 << 2);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  == SENSOR_ACTIVE_LEVEL) s |= (1 << 3);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == SENSOR_ACTIVE_LEVEL) s |= (1 << 4);

  return s;
}

uint8_t LineSensor_Count(uint8_t s)
{
  uint8_t c = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    if (s & (1 << i)) c++;
  }
  return c;
}

int16_t LineSensor_Error(uint8_t s, uint8_t *count)
{
  int16_t error = 0;
  uint8_t c = 0;

  // 权重：左负右正
  if (s & (1 << 0)) { error -= 2; c++; }
  if (s & (1 << 1)) { error -= 1; c++; }
  if (s & (1 << 2)) { error += 0; c++; }
  if (s & (1 << 3)) { error += 1; c++; }
  if (s & (1 << 4)) { error += 2; c++; }

  if (count) *count = c;
  return error;
}

void LineFollow_Task(void)
{
  uint8_t s = LineSensor_Read();
  uint8_t count = 0;
  int16_t error = LineSensor_Error(s, &count);

  // 找不到线：按上一次偏向去找线
  if (count == 0)
  {
    if (last_dir == 1)
    {
      Motor_Set(PWM_TURN_SPEED, PWM_BASE_SPEED);   // 向右找线
    }
    else if (last_dir == 2)
    {
      Motor_Set(PWM_BASE_SPEED, PWM_TURN_SPEED);   // 向左找线
    }
    else
    {
      Motor_Stop();
    }
    return;
  }

  // PID
  int16_t derivative = error - last_error;
  int32_t correction = (PID_KP * error) + (PID_KD * derivative);

  int32_t left_pwm  = PWM_BASE_SPEED - correction;
  int32_t right_pwm = PWM_BASE_SPEED + correction;

  left_pwm  = clamp_pwm(left_pwm);
  right_pwm = clamp_pwm(right_pwm);

  // 保底速度，防止太低不转
  if (left_pwm  < PWM_MIN_SPEED)  left_pwm  = PWM_MIN_SPEED;
  if (right_pwm < PWM_MIN_SPEED)  right_pwm = PWM_MIN_SPEED;

  Motor_Set((uint16_t)left_pwm, (uint16_t)right_pwm);

  if (error > 0)      last_dir = 1;
  else if (error < 0) last_dir = 2;
  else                last_dir = 0;

  last_error = error;
}

void App_Task(void)
{
  uint8_t s = LineSensor_Read();
  uint8_t marker = (LineSensor_Count(s) >= MARKER_COUNT_MIN);
  uint32_t now = HAL_GetTick();

  // 上电初始化阶段
  if (app_state == APP_INIT)
  {
    Motor_Stop();
    Buzzer_Beep3();

    state_tick = now;
    last_marker_trigger_tick = 0;
    marker_latched = 0;
    b_done = 0;

    app_state = APP_RUN_TO_B;
    return;
  }

  // 标记去抖逻辑
  if (marker)
  {
    if (!marker_latched)
    {
      marker_latched = 1;
      marker_enter_tick = now;
    }
  }
  else
  {
    marker_latched = 0;
  }

  // 路口/标记触发冷却
  uint8_t marker_ready = 0;
  if (marker_latched &&
      (now - marker_enter_tick >= MARKER_HOLD_MS) &&
      (now - last_marker_trigger_tick >= MARKER_COOLDOWN_MS))
  {
    marker_ready = 1;
    last_marker_trigger_tick = now;
  }

  switch (app_state)
  {
    case APP_RUN_TO_B:
      LineFollow_Task();

      // 起跑后一段时间再允许识别 B，防止起点误判
      if ((now - state_tick > START_IGNORE_MS) && marker_ready)
      {
        Motor_Stop();
        Buzzer_Beep3();

        b_done = 1;
        state_tick = now;
        marker_latched = 0;

        app_state = APP_RUN_TO_C;
      }
      break;

    case APP_RUN_TO_C:
      LineFollow_Task();

      // 继续沿线跑，遇到第二个标记点就到 C
      if ((now - state_tick > START_IGNORE_MS) && marker_ready)
      {
        Motor_Stop();
        Buzzer_Beep3();

        state_tick = now;
        app_state = APP_FINISH;
        marker_latched = 0;
      }
      break;

    case APP_FINISH:
      Motor_Stop();
      break;

    default:
      Motor_Stop();
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

  // 启动 PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  // 上电提示音
  Buzzer_Beep(100);

  while (1)
  {
    App_Task();
    HAL_Delay(10);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 输出初始电平 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /* 5路循迹输入 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 电机控制脚 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 蜂鸣器 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM2 PWM 输出：PA0 / PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler(void)
{
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
    HAL_Delay(100);
  }
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
