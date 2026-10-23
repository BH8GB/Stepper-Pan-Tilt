/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "a4988.h"
#include "motor.h"
#include "protocol.h"
#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "mecanum.h"
#include "gyro.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;                          /* Encoder motor 3 (16-bit, PE9/PE11) */
TIM_HandleTypeDef htim2, htim3, htim4;            /* Encoders */
TIM_HandleTypeDef htim8;                          /* PWM */
TIM_HandleTypeDef htim15;                         /* Stepper pulse */

UART_HandleTypeDef huart1;  /* USART1: Gimbal control (Desktop pattern) */
UART_HandleTypeDef huart3;  /* USART3: Motor commands (RX only) */
UART_HandleTypeDef huart8;  /* UART8:  IMU data */
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_uart8_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* ===== Gimbal (same as Desktop) ===== */
A4988_HandleTypeDef hpan_driver;
A4988_HandleTypeDef htilt_driver;
MT6701 encoder_pan;
MT6701 encoder_tilt;
float zero_pan, zero_tilt;
volatile uint8_t  circle_on;
volatile float    circle_r, circle_spd, circle_ph, circle_cx, circle_cy;

/* ===== Motor control ===== */
PID_Controller_t g_pid[4];
float g_base_speed_rpm[4];
PID_Controller_t g_yaw_pid, g_omega_pid;
volatile float g_imu_yaw;
float g_yaw_target, g_yaw_rate_cmd;
uint8_t g_yaw_hold_en, g_omega_en;
volatile uint32_t last_cmd_tick;
volatile uint32_t g_last_imu_update;
static float    prev_imu_yaw;
static uint32_t prev_imu_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM15_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
void StartDefaultTask(void *argument);
void StartMotorTask(void *argument);
void StartCommandTask(void *argument);
void StartImuTask(void *argument);

static void Motor_ParseLine(const char *line);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USART3 IMU RX callback */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3) {
    Gyro_UART_RxCallback(huart, Size);
  }
}
/* USER CODE END 0 */

int main(void)
{
  MPU_Config();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_UART8_Init();

  /* ===== Init encoders ===== */
  MT6701_Init(&encoder_pan,  &hi2c1, MT6701_I2C_ADDR_PAN);
  MT6701_Init(&encoder_tilt, &hi2c2, MT6701_I2C_ADDR_TILT);

  /* ===== Init A4988 driver ICs ===== */
  A4988_PinConfigTypeDef cfg;
  A4988_PIN_CFG(cfg, PAN_STEP_GPIO_Port, PAN_STEP_Pin, PAN_DIR_GPIO_Port, PAN_DIR_Pin,
                PAN_EN_GPIO_Port, PAN_EN_Pin, PAN_MS1_GPIO_Port, PAN_MS1_Pin,
                PAN_MS2_GPIO_Port, PAN_MS2_Pin, PAN_MS3_GPIO_Port, PAN_MS3_Pin,
                PAN_SLEEP_GPIO_Port, PAN_SLEEP_Pin, PAN_RESET_GPIO_Port, PAN_RESET_Pin);
  A4988_Init(&hpan_driver, &cfg);

  A4988_PIN_CFG(cfg, TILT_STEP_GPIO_Port, TILT_STEP_Pin, TILT_DIR_GPIO_Port, TILT_DIR_Pin,
                TILT_EN_GPIO_Port, TILT_EN_Pin, TILT_MS1_GPIO_Port, TILT_MS1_Pin,
                TILT_MS2_GPIO_Port, TILT_MS2_Pin, TILT_MS3_GPIO_Port, TILT_MS3_Pin,
                TILT_SLEEP_GPIO_Port, TILT_SLEEP_Pin, TILT_RESET_GPIO_Port, TILT_RESET_Pin);
  A4988_Init(&htilt_driver, &cfg);

  /* ===== Init motor control (open-loop) ===== */
  Motor_Init(&g_motor[MOTOR_PAN], &hpan_driver,
             PAN_GEAR_RATIO, 0, 0.0f, 0.0f);
  Motor_Init(&g_motor[MOTOR_TILT], &htilt_driver,
             TILT_GEAR_RATIO, 1, TILT_ANGLE_MIN, TILT_ANGLE_MAX);

  g_motor[MOTOR_PAN].current_angle = 0.0f;
  g_motor[MOTOR_PAN].target_angle  = 0.0f;
  g_motor[MOTOR_TILT].current_angle = 0.0f;
  g_motor[MOTOR_TILT].target_angle  = 0.0f;

  /* ===== Wake up A4988 ===== */
  A4988_Wakeup(&hpan_driver);
  A4988_Wakeup(&htilt_driver);

  /* ===== Start serial protocol + TIM15 step pulse ===== */
  Protocol_Init();
  HAL_TIM_Base_Start_IT(&htim15);

  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n[BOOT]\r\n", 9, 500);

  /* ===== Motor PWM & Encoders ===== */
  DC_Motor_Init();
  Encoder_Init();

  /* ===== PIDs ===== */
  for (int i = 0; i < 4; i++)
    PID_Init(&g_pid[i], 0.75f, 5.00f, 0.01f, 100.0f, -100.0f);
  PID_Init(&g_yaw_pid,   YAW_KP, YAW_KI, YAW_KD, YAW_OUT_MAX, YAW_OUT_MIN);
  PID_Init(&g_omega_pid, OMEGA_KP, OMEGA_KI, OMEGA_KD, OMEGA_OUT_MAX, OMEGA_OUT_MIN);

  /* ===== IMU RX (USART3 PC10/PC11) ===== */
  Gyro_Init(&huart3);


  last_cmd_tick = HAL_GetTick();

  /* Init scheduler */
  osKernelInitialize();

  defaultTaskHandle  = osThreadNew(StartDefaultTask,  NULL, &defaultTask_attributes);
  motorTaskHandle    = osThreadNew(StartMotorTask,    NULL, &motorTask_attributes);
  commandTaskHandle  = osThreadNew(StartCommandTask,  NULL, &commandTask_attributes);
  imuTaskHandle      = osThreadNew(StartImuTask,      NULL, &imuTask_attributes);

  osKernelStart();
  while (1) {}
}

/* ===== System Clock (same as Desktop) ===== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/* ===== I2C1 (same as Desktop) ===== */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

/* ===== I2C2 (same as Desktop) ===== */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) Error_Handler();
}

/* ===== USART1 (EXACT copy from Desktop) ===== */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) Error_Handler();
  CLEAR_BIT(huart1.Instance->CR3, USART_CR3_RXFTCFG | USART_CR3_TXFTCFG);
  { GPIO_InitTypeDef rx_init = {0};
    rx_init.Pin   = GPIO_PIN_10;
    rx_init.Mode  = GPIO_MODE_AF_PP;
    rx_init.Pull  = GPIO_PULLUP;
    rx_init.Speed = GPIO_SPEED_FREQ_LOW;
    rx_init.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &rx_init); }
}

/* ===== USART3 (same pattern as USART1, but PC10/PC11) ===== */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
  CLEAR_BIT(huart3.Instance->CR3, USART_CR3_RXFTCFG | USART_CR3_TXFTCFG);
  { GPIO_InitTypeDef rx_init = {0};
    rx_init.Pin   = GPIO_PIN_11;
    rx_init.Mode  = GPIO_MODE_AF_PP;
    rx_init.Pull  = GPIO_PULLUP;
    rx_init.Speed = GPIO_SPEED_FREQ_LOW;
    rx_init.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &rx_init); }
}

/* ===== UART8 (same pattern, PE0/PE1) ===== */
static void MX_UART8_Init(void)
{
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK) Error_Handler();
  CLEAR_BIT(huart8.Instance->CR3, USART_CR3_RXFTCFG | USART_CR3_TXFTCFG);
  { GPIO_InitTypeDef rx_init = {0};
    rx_init.Pin   = GPIO_PIN_0;
    rx_init.Mode  = GPIO_MODE_AF_PP;
    rx_init.Pull  = GPIO_PULLUP;
    rx_init.Speed = GPIO_SPEED_FREQ_LOW;
    rx_init.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &rx_init); }
}

/* ===== TIM15 (same as Desktop) ===== */
static void MX_TIM15_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 47;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) Error_Handler();
}

/* ===== TIM2/3/4/5 Encoders ===== */
#define MX_TIMx_ENCODER_INIT(tim, inst, period) do { \
  TIM_Encoder_InitTypeDef sCfg = {0}; TIM_MasterConfigTypeDef sMst = {0}; \
  tim.Instance = inst; tim.Init.Prescaler = 0; tim.Init.CounterMode = TIM_COUNTERMODE_UP; \
  tim.Init.Period = period; tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; \
  tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; \
  sCfg.EncoderMode = TIM_ENCODERMODE_TI12; \
  sCfg.IC1Polarity = TIM_ICPOLARITY_RISING; sCfg.IC1Selection = TIM_ICSELECTION_DIRECTTI; \
  sCfg.IC1Prescaler = TIM_ICPSC_DIV1; sCfg.IC1Filter = 0; \
  sCfg.IC2Polarity = TIM_ICPOLARITY_RISING; sCfg.IC2Selection = TIM_ICSELECTION_DIRECTTI; \
  sCfg.IC2Prescaler = TIM_ICPSC_DIV1; sCfg.IC2Filter = 0; \
  HAL_TIM_Encoder_Init(&tim, &sCfg); \
  sMst.MasterOutputTrigger = TIM_TRGO_RESET; sMst.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; \
  HAL_TIMEx_MasterConfigSynchronization(&tim, &sMst); \
} while(0)

static void MX_TIM2_Init(void) { MX_TIMx_ENCODER_INIT(htim2, TIM2, 4294967295); }
static void MX_TIM3_Init(void) { MX_TIMx_ENCODER_INIT(htim3, TIM3, 65535); }
static void MX_TIM4_Init(void) { MX_TIMx_ENCODER_INIT(htim4, TIM4, 65535); }
static void MX_TIM1_Init(void) { MX_TIMx_ENCODER_INIT(htim1, TIM1, 65535); }

/* ===== TIM8 PWM (PC6~PC9) ===== */
static void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sCC = {0}; TIM_MasterConfigTypeDef sMC = {0};
  TIM_OC_InitTypeDef sOC = {0};
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 239;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK) Error_Handler();
  sCC.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sCC) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) Error_Handler();
  sMC.MasterOutputTrigger = TIM_TRGO_RESET; sMC.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMC) != HAL_OK) Error_Handler();
  sOC.OCMode = TIM_OCMODE_PWM1; sOC.Pulse = 0;
  sOC.OCPolarity = TIM_OCPOLARITY_HIGH; sOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
  /* TIM8 is advanced timer: configure break/deadtime for MOE */
  TIM_BreakDeadTimeConfigTypeDef sBreak = {0};
  sBreak.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreak.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreak.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreak.DeadTime = 0;
  sBreak.BreakState = TIM_BREAK_DISABLE;
  sBreak.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreak.BreakFilter = 0;
  sBreak.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreak) != HAL_OK) Error_Handler();
}

/* ===== DMA (same as Desktop + DMA2) ===== */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* ===== GPIO (same as Desktop + motor pins) ===== */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PAN_EN_Pin|PAN_MS1_Pin|PAN_MS2_Pin|PAN_MS3_Pin
                          |PAN_SLEEP_Pin|PAN_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, PAN_DIR_Pin|PAN_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, TILT_DIR_Pin|TILT_STEP_Pin|TILT_EN_Pin|TILT_MS1_Pin
                          |TILT_MS2_Pin|TILT_MS3_Pin|TILT_SLEEP_Pin|TILT_RESET_Pin, GPIO_PIN_RESET);
  /* Motor direction pins */
  HAL_GPIO_WritePin(GPIOC, M1_IN1_Pin|M1_IN2_Pin|M2_IN1_Pin|M2_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, M3_IN1_Pin|M3_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, M4_IN1_Pin|M4_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PAN */
  GPIO_InitStruct.Pin = PAN_EN_Pin|PAN_MS1_Pin|PAN_MS2_Pin|PAN_MS3_Pin
                          |PAN_SLEEP_Pin|PAN_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = PAN_DIR_Pin|PAN_STEP_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pins : TILT */
  GPIO_InitStruct.Pin = TILT_DIR_Pin|TILT_STEP_Pin|TILT_EN_Pin|TILT_MS1_Pin
                          |TILT_MS2_Pin|TILT_MS3_Pin|TILT_SLEEP_Pin|TILT_RESET_Pin;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /*Configure Motor direction GPIOs */
  GPIO_InitStruct.Pin = M1_IN1_Pin|M1_IN2_Pin|M2_IN1_Pin|M2_IN2_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = M3_IN1_Pin|M3_IN2_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = M4_IN1_Pin|M4_IN2_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

  /* Fix critical pin initial levels: RESET=H, MSx=H(16 microstep), EN=H(disabled) */
  HAL_GPIO_WritePin(PAN_RESET_GPIO_Port, PAN_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAN_MS1_GPIO_Port, PAN_MS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAN_MS2_GPIO_Port, PAN_MS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAN_MS3_GPIO_Port, PAN_MS3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_RESET_GPIO_Port, TILT_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_MS1_GPIO_Port, TILT_MS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_MS2_GPIO_Port, TILT_MS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_MS3_GPIO_Port, TILT_MS3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_SET);
  /* STEP pin set to high speed */
  GPIO_InitStruct.Pin = PAN_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PAN_STEP_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = TILT_STEP_Pin;
  HAL_GPIO_Init(TILT_STEP_GPIO_Port, &GPIO_InitStruct);
}

/* ===== FreeRTOS Tasks ===== */
/* defaultTask (same as Desktop) */
void StartDefaultTask(void *argument)
{
  osDelay(200);
  for (;;) {
    osDelay(100);
    if (MT6701_Read(&encoder_pan) == HAL_OK) {
      float raw = MT6701_GetAngle(&encoder_pan);
      float diff = raw - zero_pan;
      if (diff > 180.0f) diff -= 360.0f;
      else if (diff < -180.0f) diff += 360.0f;
      float a = diff / PAN_GEAR_RATIO;
      if (g_motor[MOTOR_PAN].state == MOTOR_IDLE) {
        g_motor[MOTOR_PAN].current_angle = a;
        g_motor[MOTOR_PAN].current_step  = (int32_t)(a * g_motor[MOTOR_PAN].steps_per_degree);
      }
    }
    if (MT6701_Read(&encoder_tilt) == HAL_OK) {
      float raw = MT6701_GetAngle(&encoder_tilt);
      float diff = raw - zero_tilt;
      if (diff > 180.0f) diff -= 360.0f;
      else if (diff < -180.0f) diff += 360.0f;
      float a = diff / TILT_GEAR_RATIO;
      if (g_motor[MOTOR_TILT].state == MOTOR_IDLE) {
        g_motor[MOTOR_TILT].current_angle = a;
        g_motor[MOTOR_TILT].current_step  = (int32_t)(a * g_motor[MOTOR_TILT].steps_per_degree);
      }
    }
  }
}

/* motorTask (same as Desktop) */
void StartMotorTask(void *argument)
{
  Motor_SetTim15Priority();
  osDelay(10);
  for (int i = 0; i < 3; i++) {
    MT6701_Read(&encoder_pan);
    MT6701_Read(&encoder_tilt);
    osDelay(5);
  }
  if (MT6701_OK(&encoder_pan))  zero_pan  = MT6701_GetAngle(&encoder_pan);
  if (MT6701_OK(&encoder_tilt)) zero_tilt = MT6701_GetAngle(&encoder_tilt);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1);
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    if (circle_on) {
      float dt = 1.0f / (float)MOTOR_CTRL_FREQ;
      float omega = circle_spd * 3.14159265f / 180.0f;
      circle_ph += circle_spd * dt;
      if (circle_ph > 360.0f) circle_ph -= 360.0f;
      float rad = circle_ph * 3.14159265f / 180.0f;
      float vp = -circle_r * sinf(rad) * omega;
      float vt =  circle_r * cosf(rad) * omega;
      {
        float tp = Motor_GetAngle(&g_motor[MOTOR_PAN])  + vp * dt;
        float tt = Motor_GetAngle(&g_motor[MOTOR_TILT]) + vt * dt;
        if (tt < TILT_ANGLE_MIN || tt > TILT_ANGLE_MAX) vt = -vt;
      }
      Motor_Spin(&g_motor[MOTOR_PAN],  vp);
      Motor_Spin(&g_motor[MOTOR_TILT], vt);
    }
    Motor_Update(&g_motor[MOTOR_PAN]);
    Motor_Update(&g_motor[MOTOR_TILT]);
  }
}

/* commandTask (same as Desktop + USART3 motor parser) */
void StartCommandTask(void *argument)
{
  osDelay(20);
  Protocol_SendWelcome();
  TickType_t xLastMon = xTaskGetTickCount();
  for (;;) {
    Protocol_Poll();
    Protocol_CheckTimeout();
    if (Protocol_LineReady()) Protocol_ProcessLine();

    if (g_monitor && (xTaskGetTickCount() - xLastMon) >= pdMS_TO_TICKS(500)) {
      xLastMon = xTaskGetTickCount();
      Protocol_SendStatus();
    }
    osDelay(1);
  }
}

/* imuTask */
void StartImuTask(void *argument)
{
  osDelay(100);
  /* Start UART8 DMA circular TX for processed IMU data */
  static char imu_tx_buf[64];
  HAL_UART_Transmit_DMA(&huart8, (uint8_t*)imu_tx_buf, sizeof(imu_tx_buf));
  for (;;) {
    if (Gyro_AngleUpdated()) {
      g_imu_yaw = Gyro_GetYaw();
      g_last_imu_update = HAL_GetTick();
    }
    if (Gyro_RateUpdated()) { Gyro_GetRateZ(); }
    Gyro_ClearFlags();
    /* Format and update DMA TX buffer with latest IMU data */
    int len = snprintf(imu_tx_buf, 64, "Y:%.1f R:%.1f\r\n", g_imu_yaw, Gyro_GetRateZ());
    if (len < 0) len = 0;
    for (int i = len; i < 64; i++) imu_tx_buf[i] = ' ';
    osDelay(10);
  }
}

/* ===== Interrupt Callbacks ===== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();

    /* 100ms PID cycle */
    static uint16_t pid_tick = 0;
    pid_tick++;
    if (pid_tick >= 100) {
      pid_tick = 0;
      Encoder_Update_Speed();

      bool imu_timeout = (HAL_GetTick() - g_last_imu_update) > 100;
      if (imu_timeout) { g_omega_en = 0; g_yaw_hold_en = 0; PID_Reset(&g_omega_pid); PID_Reset(&g_yaw_pid); }

      for (int i = 0; i < 4; i++) g_pid[i].target = g_base_speed_rpm[i];

      if (!imu_timeout) {
        if (g_omega_en) {
          float actual_omega = 0.0f; uint32_t now = HAL_GetTick();
          if (prev_imu_tick != 0) {
            float dt = (now - prev_imu_tick) / 1000.0f;
            if (dt > 0.02f && dt < 0.5f) {
              float delta = g_imu_yaw - prev_imu_yaw;
              while (delta > 180.0f) delta -= 360.0f;
              while (delta < -180.0f) delta += 360.0f;
              actual_omega = (delta * 3.14159265358979f / 180.0f) / dt;
            }
          }
          prev_imu_yaw = g_imu_yaw; prev_imu_tick = now;
          if (fabsf(actual_omega) > 50.0f) actual_omega = 0.0f;
          g_omega_pid.target = g_yaw_rate_cmd;
          float omega_out = PID_Compute(&g_omega_pid, actual_omega);
          if (omega_out > OMEGA_OUT_MAX) omega_out = OMEGA_OUT_MAX;
          if (omega_out < OMEGA_OUT_MIN) omega_out = OMEGA_OUT_MIN;
          g_pid[0].target -= omega_out; g_pid[2].target -= omega_out;
          g_pid[1].target += omega_out; g_pid[3].target += omega_out;
        } else if (g_yaw_hold_en) {
          g_yaw_pid.target = g_yaw_target;
          float yaw_output = PID_Compute(&g_yaw_pid, g_imu_yaw);
          float yaw_error = g_yaw_target - g_imu_yaw;
          while (yaw_error > 180.0f) yaw_error -= 360.0f;
          while (yaw_error < -180.0f) yaw_error += 360.0f;
          if (fabsf(yaw_error) > 30.0f) g_yaw_pid.integral = 0.0f;
          if (fabsf(yaw_error) < 2.0f) { yaw_output = 0.0f; g_yaw_pid.integral = 0.0f; }
          if (yaw_output > YAW_OUT_MAX) yaw_output = YAW_OUT_MAX;
          if (yaw_output < YAW_OUT_MIN) yaw_output = YAW_OUT_MIN;
          g_pid[0].target -= yaw_output; g_pid[2].target -= yaw_output;
          g_pid[1].target += yaw_output; g_pid[3].target += yaw_output;
        }
        for (int i = 0; i < 4; i++) {
          if (g_pid[i].target > 300.0f) g_pid[i].target = 300.0f;
          if (g_pid[i].target < -300.0f) g_pid[i].target = -300.0f;
        }
      }
      for (int i = 0; i < 4; i++) {
        float out;
        if (g_base_speed_rpm[i] == 0.0f) {
            out = 0.0f;
            PID_Reset(&g_pid[i]);
        } else {
            /* PID only adjusts ±10 around base speed */
            out = PID_Compute(&g_pid[i], g_motor_encoders[i].speed_rpm);
            if (out > g_base_speed_rpm[i] + 10.0f) out = g_base_speed_rpm[i] + 10.0f;
            if (out < g_base_speed_rpm[i] - 10.0f) out = g_base_speed_rpm[i] - 10.0f;
            if (out > 100.0f) out = 100.0f;
            if (out < -100.0f) out = -100.0f;
        }
        DC_Motor_SetSpeed(i, (int16_t)out);
      }
    }
    return;
  }

  /* Encoder overflow (TIM2/3/4/5) */
  int motor_id = -1;
  if (htim->Instance == TIM2) motor_id = 0;
  else if (htim->Instance == TIM3) motor_id = 1;
  else if (htim->Instance == TIM4) motor_id = 2;
  else if (htim->Instance == TIM1) motor_id = 3;
  if (motor_id >= 0) {
    uint64_t step = (htim->Instance == TIM2) ? 0x100000000ULL : 65536ULL;
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      g_motor_encoders[motor_id].total_pulse -= (int64_t)step;
    else
      g_motor_encoders[motor_id].total_pulse += (int64_t)step;
  }
}

/* MPU Configuration (same as Desktop) */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  __disable_irq();
  Motor_Stop(&g_motor[MOTOR_PAN]);
  Motor_Stop(&g_motor[MOTOR_TILT]);
  for (int i = 0; i < 4; i++) DC_Motor_SetSpeed(i, 0);
  while (1) {}
}
