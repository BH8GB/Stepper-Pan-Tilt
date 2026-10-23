/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "../../MDK-ARM/EricE/mt6701.h"
#include "../../MDK-ARM/EricE/a4988.h"
#include "../../MDK-ARM/EricE/motor.h"
#include "../../MDK-ARM/EricE/protocol.h"
#include "../../MDK-ARM/EricE/motor_control.h"
#include "../../MDK-ARM/EricE/encoder.h"
#include "../../MDK-ARM/EricE/pid.h"
#include "../../MDK-ARM/EricE/mecanum.h"
#include "../../MDK-ARM/EricE/gyro.h"
#include "../../MDK-ARM/EricE/servo_feedback.h"
/* USER CODE END Includes */

void Error_Handler(void);
/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PAN_EN_Pin GPIO_PIN_0
#define PAN_EN_GPIO_Port GPIOA
#define PAN_MS1_Pin GPIO_PIN_2
#define PAN_MS1_GPIO_Port GPIOA
#define PAN_MS2_Pin GPIO_PIN_3
#define PAN_MS2_GPIO_Port GPIOA
#define PAN_MS3_Pin GPIO_PIN_4
#define PAN_MS3_GPIO_Port GPIOA
#define PAN_SLEEP_Pin GPIO_PIN_5
#define PAN_SLEEP_GPIO_Port GPIOA
#define PAN_RESET_Pin GPIO_PIN_6
#define PAN_RESET_GPIO_Port GPIOA
#define PAN_DIR_Pin GPIO_PIN_0
#define PAN_DIR_GPIO_Port GPIOB
#define PAN_STEP_Pin GPIO_PIN_1
#define PAN_STEP_GPIO_Port GPIOB
#define TILT_DIR_Pin GPIO_PIN_0
#define TILT_DIR_GPIO_Port GPIOD
#define TILT_STEP_Pin GPIO_PIN_1
#define TILT_STEP_GPIO_Port GPIOD
#define TILT_EN_Pin GPIO_PIN_2
#define TILT_EN_GPIO_Port GPIOD
#define TILT_MS1_Pin GPIO_PIN_3
#define TILT_MS1_GPIO_Port GPIOD
#define TILT_MS2_Pin GPIO_PIN_4
#define TILT_MS2_GPIO_Port GPIOD
#define TILT_MS3_Pin GPIO_PIN_5
#define TILT_MS3_GPIO_Port GPIOD
#define TILT_SLEEP_Pin GPIO_PIN_6
#define TILT_SLEEP_GPIO_Port GPIOD
#define TILT_RESET_Pin GPIO_PIN_7
#define TILT_RESET_GPIO_Port GPIOD

/* Motor direction pins */
#define M1_IN1_Pin GPIO_PIN_0
#define M1_IN1_GPIO_Port GPIOC
#define M1_IN2_Pin GPIO_PIN_1
#define M1_IN2_GPIO_Port GPIOC
#define M2_IN1_Pin GPIO_PIN_2
#define M2_IN1_GPIO_Port GPIOC
#define M2_IN2_Pin GPIO_PIN_3
#define M2_IN2_GPIO_Port GPIOC
#define M3_IN1_Pin GPIO_PIN_11
#define M3_IN1_GPIO_Port GPIOA
#define M3_IN2_Pin GPIO_PIN_12
#define M3_IN2_GPIO_Port GPIOA
#define M4_IN1_Pin GPIO_PIN_12
#define M4_IN1_GPIO_Port GPIOB
#define M4_IN2_Pin GPIO_PIN_13
#define M4_IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define I2C1_SCL_PIN        GPIO_PIN_8
#define I2C1_SCL_PORT       GPIOB
#define I2C1_SDA_PIN        GPIO_PIN_7
#define I2C1_SDA_PORT       GPIOB
#define USART1_TX_PIN       GPIO_PIN_9
#define USART1_TX_PORT      GPIOA
#define USART1_RX_PIN       GPIO_PIN_10
#define USART1_RX_PORT      GPIOA
#define PAN_DIR_PORT        GPIOB
#define PAN_DIR_PIN         GPIO_PIN_0
#define PAN_STEP_PORT       GPIOB
#define PAN_STEP_PIN        GPIO_PIN_1
#define PAN_EN_PORT         GPIOA
#define PAN_EN_PIN          GPIO_PIN_0
#define PAN_MS1_PORT        GPIOA
#define PAN_MS1_PIN         GPIO_PIN_2
#define PAN_MS2_PORT        GPIOA
#define PAN_MS2_PIN         GPIO_PIN_3
#define PAN_MS3_PORT        GPIOA
#define PAN_MS3_PIN         GPIO_PIN_4
#define PAN_SLEEP_PORT      GPIOA
#define PAN_SLEEP_PIN       GPIO_PIN_5
#define PAN_RESET_PORT      GPIOA
#define PAN_RESET_PIN       GPIO_PIN_6
#define TILT_DIR_PORT       GPIOD
#define TILT_DIR_PIN        GPIO_PIN_0
#define TILT_STEP_PORT      GPIOD
#define TILT_STEP_PIN       GPIO_PIN_1
#define TILT_EN_PORT        GPIOD
#define TILT_EN_PIN         GPIO_PIN_2
#define TILT_MS1_PORT       GPIOD
#define TILT_MS1_PIN        GPIO_PIN_3
#define TILT_MS2_PORT       GPIOD
#define TILT_MS2_PIN        GPIO_PIN_4
#define TILT_MS3_PORT       GPIOD
#define TILT_MS3_PIN        GPIO_PIN_5
#define TILT_SLEEP_PORT     GPIOD
#define TILT_SLEEP_PIN      GPIO_PIN_6
#define TILT_RESET_PORT     GPIOD
#define TILT_RESET_PIN      GPIO_PIN_7

extern I2C_HandleTypeDef  hi2c1, hi2c2;
extern UART_HandleTypeDef huart1;   /* USART1: Gimbal */
extern UART_HandleTypeDef huart3;   /* USART3: Motor control */
extern UART_HandleTypeDef huart8;   /* UART8:  IMU data */
extern TIM_HandleTypeDef  htim1;    /* Encoder motor 3 (16-bit, PE9/PE11) */
extern TIM_HandleTypeDef  htim2, htim3, htim4; /* Encoders */
extern TIM_HandleTypeDef  htim8;    /* PWM */
extern TIM_HandleTypeDef  htim15;   /* Stepper pulse */
extern DMA_HandleTypeDef  hdma_usart3_rx, hdma_usart3_tx, hdma_uart8_rx;

extern osThreadId_t motorTaskHandle, commandTaskHandle;
extern osThreadId_t imuTaskHandle, feedbackTaskHandle;

extern PID_Controller_t g_pid[4], g_yaw_pid, g_omega_pid;
extern float g_base_speed_rpm[4];
extern volatile float g_imu_yaw;
extern float g_yaw_target, g_yaw_rate_cmd;
extern uint8_t g_yaw_hold_en, g_omega_en;
extern volatile uint32_t last_cmd_tick;
extern volatile uint8_t dma_tx_busy;

#define YAW_KP 0.35f
#define YAW_KI 0.008f
#define YAW_KD 0.02f
#define YAW_OUT_MAX 80.0f
#define YAW_OUT_MIN -80.0f
#define OMEGA_KP 0.55f
#define OMEGA_KI 0.05f
#define OMEGA_KD 0.01f
#define OMEGA_OUT_MAX 30.0f
#define OMEGA_OUT_MIN -30.0f

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
