/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        MSP Initialization and de-Initialization codes.
  *               I2C1, USART1, TIM15, TIM2-5, TIM8, USART3, UART8
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
/* USER CODE END Define */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */
/* USER CODE END Macro */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */
/* USER CODE END ExternalFunctions */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */
  /* USER CODE END MspInit 0 */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /* System interrupt init*/
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
  /* USER CODE BEGIN MspInit 1 */
  /* USER CODE END MspInit 1 */
}
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
    /* USER CODE BEGIN I2C1_MspInit 0 */
    /* USER CODE END I2C1_MspInit 0 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    __HAL_RCC_I2C1_CLK_ENABLE();
    hdma_i2c1_rx.Instance = DMA1_Stream1;
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_I2C1_RX;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(hi2c,hdmarx,hdma_i2c1_rx);
    hdma_i2c1_tx.Instance = DMA1_Stream2;
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_I2C1_TX;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_i2c1_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2c1_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_i2c1_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(hi2c,hdmatx,hdma_i2c1_tx);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0); HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0); HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    /* USER CODE BEGIN I2C1_MspInit 1 */
    /* USER CODE END I2C1_MspInit 1 */
  }
  else if(hi2c->Instance==I2C2)
  {
    /* USER CODE BEGIN I2C2_MspInit 0 */
    /* USER CODE END I2C2_MspInit 0 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    __HAL_RCC_I2C2_CLK_ENABLE();
    hdma_i2c2_rx.Instance = DMA1_Stream4;
    hdma_i2c2_rx.Init.Request = DMA_REQUEST_I2C2_RX;
    hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c2_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(hi2c,hdmarx,hdma_i2c2_rx);
    hdma_i2c2_tx.Instance = DMA1_Stream5;
    hdma_i2c2_tx.Init.Request = DMA_REQUEST_I2C2_TX;
    hdma_i2c2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c2_tx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(hi2c,hdmatx,hdma_i2c2_tx);
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0); HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0); HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    /* USER CODE BEGIN I2C2_MspInit 1 */
    /* USER CODE END I2C2_MspInit 1 */
  }
}
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1) {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7); HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
    HAL_DMA_DeInit(hi2c->hdmarx); HAL_DMA_DeInit(hi2c->hdmatx);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn); HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  }
  else if(hi2c->Instance==I2C2) {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10); HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
    HAL_DMA_DeInit(hi2c->hdmarx); HAL_DMA_DeInit(hi2c->hdmatx);
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn); HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
  }
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM15)
  {
    /* USER CODE BEGIN TIM15_MspInit 0 */
    /* USER CODE END TIM15_MspInit 0 */
    __HAL_RCC_TIM15_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM15_IRQn, 5, 0); HAL_NVIC_EnableIRQ(TIM15_IRQn);
    /* USER CODE BEGIN TIM15_MspInit 1 */
    /* USER CODE END TIM15_MspInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0); HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
  else if(htim_base->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0); HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
  else if(htim_base->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0); HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
  else if(htim_base->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 5, 0); HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  }
  else if(htim_base->Instance==TIM8)
  {
    __HAL_RCC_TIM8_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 5, 0); HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 5, 0); HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  }
}
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM15)
  {
    /* USER CODE BEGIN TIM15_MspDeInit 0 */
    /* USER CODE END TIM15_MspDeInit 0 */
    __HAL_RCC_TIM15_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM15_IRQn);
    /* USER CODE BEGIN TIM15_MspDeInit 1 */
    /* USER CODE END TIM15_MspDeInit 1 */
  }
}

/* ===== TIM Encoder MSP (TIM2/3/4/5) ===== */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_HIGH;
  if (htim->Instance == TIM2) {
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = GPIO_PIN_15; g.Alternate = GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_3;  HAL_GPIO_Init(GPIOB, &g);
  } else if (htim->Instance == TIM3) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = GPIO_PIN_4|GPIO_PIN_5; g.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOB, &g);
  } else if (htim->Instance == TIM4) {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    g.Pin = GPIO_PIN_12|GPIO_PIN_13; g.Alternate = GPIO_AF2_TIM4; HAL_GPIO_Init(GPIOD, &g);
  } else if (htim->Instance == TIM1) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    g.Pin = GPIO_PIN_9|GPIO_PIN_11; g.Alternate = GPIO_AF1_TIM1; HAL_GPIO_Init(GPIOE, &g);
  }
}

/* ===== TIM PWM MSP (TIM8, PC6~PC9 AF3) ===== */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM8) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &g);
  }
}

/* ===== UART MSP ===== */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */
    /* USER CODE END USART1_MspInit 0 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    hdma_usart1_rx.Instance = DMA1_Stream0;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE; hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR; hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(huart,hdmarx,hdma_usart1_rx);
    hdma_usart1_tx.Instance = DMA1_Stream3;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE; hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_CIRCULAR; hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(huart,hdmatx,hdma_usart1_tx);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0); HAL_NVIC_EnableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspInit 1 */
    /* USER CODE END USART1_MspInit 1 */
  }
  else if(huart->Instance==USART3)
  {
    /* Mirror USART1 pattern for PC10/PC11 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    hdma_usart3_rx.Instance = DMA1_Stream6;
    hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE; hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR; hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(huart,hdmarx,hdma_usart3_rx);
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0); HAL_NVIC_EnableIRQ(USART3_IRQn);
  }
  else if(huart->Instance==UART8)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART8;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
    __HAL_RCC_UART8_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    /* UART8 TX DMA (DMA2_Stream0, CIRCULAR) for processed IMU data */
    extern DMA_HandleTypeDef hdma_uart8_tx;
    hdma_uart8_tx.Instance = DMA2_Stream0;
    hdma_uart8_tx.Init.Request = DMA_REQUEST_UART8_TX;
    hdma_uart8_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart8_tx.Init.PeriphInc = DMA_PINC_DISABLE; hdma_uart8_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart8_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart8_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart8_tx.Init.Mode = DMA_CIRCULAR; hdma_uart8_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart8_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart8_tx) != HAL_OK) { Error_Handler(); }
    __HAL_LINKDMA(huart,hdmatx,hdma_uart8_tx);
    HAL_NVIC_SetPriority(UART8_IRQn, 5, 0); HAL_NVIC_EnableIRQ(UART8_IRQn);
  }
}
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */
    /* USER CODE END USART1_MspDeInit 0 */
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
    HAL_DMA_DeInit(huart->hdmarx); HAL_DMA_DeInit(huart->hdmatx);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspDeInit 1 */
    /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(huart->Instance==USART3) {
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
  else if(huart->Instance==UART8) {
    __HAL_RCC_UART8_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0|GPIO_PIN_1);
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_NVIC_DisableIRQ(UART8_IRQn);
  }
}
/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
