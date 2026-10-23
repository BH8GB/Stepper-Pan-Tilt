/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../MDK-ARM/EricE/motor.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart8;
/* USER CODE BEGIN EV */
/* USER CODE END EV */
/******************************************************************************/
void NMI_Handler(void)           { while (1) {} }
void HardFault_Handler(void)     { while (1) {} }
void MemManage_Handler(void)     { while (1) {} }
void BusFault_Handler(void)      { while (1) {} }
void UsageFault_Handler(void)    { while (1) {} }
void DebugMon_Handler(void)      { }
void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_rx); }
void DMA1_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2c1_rx); }
void DMA1_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2c1_tx); }
void DMA1_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void DMA1_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2c2_rx); }
void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2c2_tx); }
void DMA1_Stream6_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart3_rx); }
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_uart8_tx); }
void I2C1_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c1); }
void I2C2_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c2); }
void I2C2_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c2); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }
void UART8_IRQHandler(void)  { HAL_UART_IRQHandler(&huart8); }
void TIM1_UP_TIM10_IRQHandler(void)  { HAL_TIM_IRQHandler(&htim1); }
void TIM2_IRQHandler(void)           { HAL_TIM_IRQHandler(&htim2); }
void TIM3_IRQHandler(void)           { HAL_TIM_IRQHandler(&htim3); }
void TIM4_IRQHandler(void)           { HAL_TIM_IRQHandler(&htim4); }
void TIM6_DAC_IRQHandler(void)       { HAL_TIM_IRQHandler(&htim6); }
void TIM8_UP_TIM13_IRQHandler(void)  { HAL_TIM_IRQHandler(&htim8); }
void TIM8_BRK_TIM12_IRQHandler(void) { HAL_TIM_IRQHandler(&htim8); }
void TIM15_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim15);
  Motor_StepISR(&g_motor[MOTOR_PAN]);
  Motor_StepISR(&g_motor[MOTOR_TILT]);
}
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
