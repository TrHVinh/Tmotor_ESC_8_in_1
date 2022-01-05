/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ringbuf.h"
#include "telemetry_rtr.h"
#include "softuart.h"
//#include "mavlink/telemetry/mavlink.h"
#include "c_library_v2-master/ardupilotmega/mavlink.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE        1
#define RINGBUF_BUFF_LEN    100
#define SYSTEM_ID           1
#define COMPONENT_ID        0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
static volatile RINGBUF *ringbuf0;
static volatile RINGBUF *ringbuf1;
static volatile RINGBUF *ringbuf2;
static volatile RINGBUF *ringbuf3;
static volatile RINGBUF *ringbuf4;
static volatile RINGBUF *ringbuf5;
static volatile RINGBUF *ringbuf6;
static volatile RINGBUF *ringbuf7;
TelemetryType telemetry0, telemetry1, telemetry2, telemetry3, telemetry4, telemetry5, telemetry6, telemetry7, TelemetryArray[8];
ReceiveStatusTyte recStatus0, recStatus1, recStatus2, recStatus3, recStatus4, recStatus5, recStatus6, recStatus7;
RINGBUF ring_0, ring_1, ring_2, ring_3, ring_4, ring_5, ring_6, ring_7;
osThreadId ESC0, ESC1, ESC2, ESC3, ESC4, ESC5, ESC6, ESC7;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void Parse_ESC0(void const * parameter);
void Parse_ESC1(void const * parameter);
void Parse_ESC2(void const * parameter);
void Parse_ESC3(void const * parameter);
void Parse_ESC4(void const * parameter);
void Parse_ESC5(void const * parameter);
void Parse_ESC6(void const * parameter);
void Parse_ESC7(void const * parameter);

uint8_t aRxBuffer_0, aRxBuffer_1, aRxBuffer_2, aRxBuffer_3, aRxBuffer_4, aRxBuffer_5, aRxBuffer_6, aRxBuffer_7;
uint8_t ptBufferUART0[RINGBUF_BUFF_LEN], ptBufferUART1[RINGBUF_BUFF_LEN], ptBufferUART2[RINGBUF_BUFF_LEN], ptBufferUART3[RINGBUF_BUFF_LEN],
		ptBufferUART4[RINGBUF_BUFF_LEN], ptBufferUART5[RINGBUF_BUFF_LEN], ptBufferUART6[RINGBUF_BUFF_LEN], ptBufferUART7[RINGBUF_BUFF_LEN];
uint8_t c0,c1,c2,c3,c4,c5,c6,c7;
uint8_t PWM=0;
char a[20], b[20];
int failESC0 = 0;
int failESC1 = 0;
int failESC2 = 0;
int failESC3 = 0;
int failESC4 = 0;
int failESC5 = 0;
int failESC6 = 0;
int failESC7 = 0;
int successESC0 = 0;
int successESC1 = 0;
int successESC2 = 0;
int successESC3 = 0;
int successESC4 = 0;
int successESC5 = 0;
int successESC6 = 0;
int successESC7 = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t getdata(uint8_t SoftUartNumber)
{
    uint8_t ch;
    while(SoftUartRxAlavailable(SoftUartNumber)==0);
    SoftUartReadRxBuffer(SoftUartNumber,&ch,1);
    return ch;
}

mavlink_esc_telemetry_1_to_4_t esc1 = {0};
mavlink_esc_telemetry_5_to_8_t esc2 = {0};
void transmit_mavlink_esc_1_to_4(mavlink_esc_telemetry_1_to_4_t esc){
    mavlink_message_t msg;
    uint8_t _buffer[100]= {0};
    for (int i = 0; i<4; i++){
        esc.voltage[i] = TelemetryArray[i].voltage;
        esc.count[i] = TelemetryArray[i].BaleNo;
        esc.totalcurrent[i] = TelemetryArray[i].current;
        esc.rpm[i] = TelemetryArray[i].rpm;
        esc.current[i] = TelemetryArray[i].phase;
    }
    mavlink_msg_esc_telemetry_1_to_4_encode(SYSTEM_ID, COMPONENT_ID, &msg, &esc);
    uint8_t len = mavlink_msg_to_send_buffer(_buffer, &msg);
//    SoftUartPuts(0, _buffer, len);
    HAL_UART_Transmit(&huart4, _buffer, len, 100);
//    HAL_UART_Transmit_IT(&huart4, _buffer, len);
}
void transmit_mavlink_esc_5_to_8(mavlink_esc_telemetry_5_to_8_t esc){
    mavlink_message_t msg;
    uint8_t _buffer[100]= {0};
    for (int i = 0; i<4; i++){
        esc.voltage[i] = TelemetryArray[i+4].voltage;
        esc.count[i] = TelemetryArray[i+4].BaleNo;
        esc.totalcurrent[i] = TelemetryArray[i+4].current;
        esc.rpm[i] = TelemetryArray[i+4].rpm;
        esc.current[i] = TelemetryArray[i+4].phase;
    }
    mavlink_msg_esc_telemetry_5_to_8_encode(SYSTEM_ID, COMPONENT_ID, &msg, &esc);
    uint8_t len = mavlink_msg_to_send_buffer(_buffer, &msg);
//    SoftUartPuts(0, _buffer, len);
    HAL_UART_Transmit(&huart4, _buffer, len, 100);
//    HAL_UART_Transmit_IT(&huart4, _buffer, len);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,600);
  HAL_UART_Receive_IT(&huart1, &aRxBuffer_1, RXBUFFERSIZE);
  HAL_UART_Receive_IT(&huart2, &aRxBuffer_2, RXBUFFERSIZE);
  HAL_UART_Receive_IT(&huart3, &aRxBuffer_3, RXBUFFERSIZE);
  HAL_UART_Receive_IT(&huart4, &aRxBuffer_4, RXBUFFERSIZE);
  HAL_UART_Receive_IT(&huart5, &aRxBuffer_5, RXBUFFERSIZE);
  HAL_UART_Receive_IT(&huart6, &aRxBuffer_6, RXBUFFERSIZE);
  HAL_TIM_Base_Start_IT(&htim10);

  ringbuf0 = &ring_0;
  ringbuf1 = &ring_1;
  ringbuf2 = &ring_2;
  ringbuf3 = &ring_3;
  ringbuf4 = &ring_4;
  ringbuf5 = &ring_5;
  ringbuf6 = &ring_6;
  ringbuf7 = &ring_7;

  RINGBUF_Init((RINGBUF*)ringbuf0, ptBufferUART0, RINGBUF_BUFF_LEN);
  RINGBUF_Init((RINGBUF*)ringbuf1, ptBufferUART1, RINGBUF_BUFF_LEN);
  RINGBUF_Init((RINGBUF*)ringbuf2, ptBufferUART2, RINGBUF_BUFF_LEN);
  RINGBUF_Init((RINGBUF*)ringbuf4, ptBufferUART4, RINGBUF_BUFF_LEN);
  RINGBUF_Init((RINGBUF*)ringbuf3, ptBufferUART3, RINGBUF_BUFF_LEN);
  RINGBUF_Init((RINGBUF*)ringbuf5, ptBufferUART5, RINGBUF_BUFF_LEN);
  RINGBUF_Init((RINGBUF*)ringbuf6, ptBufferUART6, RINGBUF_BUFF_LEN);
  RINGBUF_Init((RINGBUF*)ringbuf7, ptBufferUART7, RINGBUF_BUFF_LEN);

  SoftUartInit(0, SU0_TX_GPIO_Port, SU0_TX_Pin, SU0_RX_GPIO_Port, SU0_RX_Pin);
  SoftUartInit(1, SU1_TX_GPIO_Port, SU1_TX_Pin, SU1_RX_GPIO_Port, SU1_RX_Pin);
  SoftUartInit(2, SU2_TX_GPIO_Port, SU2_TX_Pin, SU2_RX_GPIO_Port, SU2_RX_Pin);
//  SoftUartInit(3, SU4_TX_GPIO_Port, SU4_TX_Pin, SU4_RX_GPIO_Port, SU4_RX_Pin);
//  SoftUartInit(5, SU5_TX_GPIO_Port, SU5_TX_Pin, SU5_RX_GPIO_Port, SU5_RX_Pin);
//  SoftUartInit(4, SU7_TX_GPIO_Port, SU7_TX_Pin, SU7_RX_GPIO_Port, SU7_RX_Pin);

  SoftUartEnableRx(0);
  SoftUartEnableRx(1);
  SoftUartEnableRx(2);
//  SoftUartEnableRx(3);
//  SoftUartEnableRx(4);
//  SoftUartEnableRx(5);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
// osThreadDef(Task0, Parse_ESC0, osPriorityHigh, 0, 128);
// ESC0 = osThreadCreate(osThread(Task0), NULL);
  osThreadDef(Task1, Parse_ESC1, osPriorityHigh, 0, 128);
  ESC1 = osThreadCreate(osThread(Task1), NULL);
  osThreadDef(Task2, Parse_ESC2, osPriorityHigh, 0, 512);
  ESC2 = osThreadCreate(osThread(Task2), NULL);
//  osThreadDef(Task3, Parse_ESC3, osPriorityHigh, 0, 256);
//  ESC3 = osThreadCreate(osThread(Task3), NULL);
//  osThreadDef(Task4, Parse_ESC4, osPriorityNormal, 0, 128);
//  ESC4 = osThreadCreate(osThread(Task4), NULL);
//  osThreadDef(Task5, Parse_ESC5, osPriorityNormal, 0, 128);
//  ESC5 = osThreadCreate(osThread(Task5), NULL);
//  osThreadDef(Task6, Parse_ESC6, osPriorityNormal, 0, 128);
//  ESC6 = osThreadCreate(osThread(Task6), NULL);
//  osThreadDef(Task7, Parse_ESC7, osPriorityHigh, 0, 128);
//  ESC7 = osThreadCreate(osThread(Task7), NULL);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 167;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 175;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 19200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 19200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 19200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SU0_TX_GPIO_Port, SU0_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|SU6_TX_Pin|SU7_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SU1_TX_GPIO_Port, SU1_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SU3_TX_Pin|SU2_TX_Pin|SU4_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SU5_TX_GPIO_Port, SU5_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SU0_TX_Pin */
  GPIO_InitStruct.Pin = SU0_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SU0_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SU0_RX_Pin SU2_RX_Pin SU6_RX_Pin SU7_RX_Pin */
  GPIO_InitStruct.Pin = SU0_RX_Pin|SU2_RX_Pin|SU6_RX_Pin|SU7_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 SU6_TX_Pin SU7_TX_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_6|SU6_TX_Pin|SU7_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SU1_TX_Pin */
  GPIO_InitStruct.Pin = SU1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SU1_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SU1_RX_Pin SU3_RX_Pin */
  GPIO_InitStruct.Pin = SU1_RX_Pin|SU3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SU3_TX_Pin SU2_TX_Pin SU4_TX_Pin */
  GPIO_InitStruct.Pin = SU3_TX_Pin|SU2_TX_Pin|SU4_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SU4_RX_Pin */
  GPIO_InitStruct.Pin = SU4_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SU4_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SU5_RX_Pin */
  GPIO_InitStruct.Pin = SU5_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SU5_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SU5_TX_Pin */
  GPIO_InitStruct.Pin = SU5_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SU5_TX_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
/* ESC 1 ---------------------------------------------------------*/
	if (HAL_UART_Receive_IT(&huart1, &aRxBuffer_1, RXBUFFERSIZE) == HAL_OK) {
		RINGBUF_Put((RINGBUF*) ringbuf1, aRxBuffer_1);
		RINGBUF_Get((RINGBUF*) ringbuf1, &c1);
		int x1 = telemetryParse(c1, &telemetry1, &recStatus1);
		if (x1 == PARSE_OK){
			successESC1++;
			TelemetryArray[1] = telemetry1;
		}
		else if (x1 == PARSE_ERROR){
			failESC1++;
		}
	}
/* ESC 2 ---------------------------------------------------------*/
	if (HAL_UART_Receive_IT(&huart2, &aRxBuffer_2, RXBUFFERSIZE) == HAL_OK) {
		RINGBUF_Put((RINGBUF*) ringbuf2, aRxBuffer_2);
		RINGBUF_Get((RINGBUF*) ringbuf2, &c2);
		int x2 = telemetryParse(c2, &telemetry2, &recStatus2);
		if (x2 == PARSE_OK){
			successESC2++;
			TelemetryArray[2] = telemetry2;
		}
		else if (x2 == PARSE_ERROR){
			failESC2++;
		}
	}
/* ESC 3 ---------------------------------------------------------*/
	if (HAL_UART_Receive_IT(&huart3, &aRxBuffer_3, RXBUFFERSIZE) == HAL_OK) {
		RINGBUF_Put((RINGBUF*) ringbuf3, aRxBuffer_3);
		RINGBUF_Get((RINGBUF*) ringbuf3, &c3);
		int x3 = telemetryParse(c3, &telemetry3, &recStatus3);
		if (x3 == PARSE_OK){
			successESC3++;
			TelemetryArray[3] = telemetry3;
		}
		else if (x3 == PARSE_ERROR){
			failESC3++;
		}
	}
/* ESC 4 ---------------------------------------------------------*/
	if (HAL_UART_Receive_IT(&huart4, &aRxBuffer_4, RXBUFFERSIZE) == HAL_OK) {
		RINGBUF_Put((RINGBUF*) ringbuf4, aRxBuffer_4);
		RINGBUF_Get((RINGBUF*) ringbuf4, &c4);
		int x4 = telemetryParse(c4, &telemetry4, &recStatus4);
		if (x4 == PARSE_OK){
			successESC4++;
			TelemetryArray[4] = telemetry4;
		}
		else if (x4 == PARSE_ERROR){
			failESC4++;
		}
	}
/* ESC 5 ---------------------------------------------------------*/
	if (HAL_UART_Receive_IT(&huart5, &aRxBuffer_5, RXBUFFERSIZE) == HAL_OK) {
		RINGBUF_Put((RINGBUF*) ringbuf5, aRxBuffer_5);
		RINGBUF_Get((RINGBUF*) ringbuf5, &c5);
		int x5 = telemetryParse(c5, &telemetry5, &recStatus5);
		if (x5 == PARSE_OK){
			successESC5++;
			TelemetryArray[5] = telemetry5;
		}
		else if (x5 == PARSE_ERROR){
			failESC5++;
		}
	}
/* ESC 6 ---------------------------------------------------------*/
  if (HAL_UART_Receive_IT(&huart6, &aRxBuffer_6, RXBUFFERSIZE) == HAL_OK) {
	  RINGBUF_Put((RINGBUF*) ringbuf6, aRxBuffer_6);
	  RINGBUF_Get((RINGBUF*) ringbuf6, &c6);
	  int x6 = telemetryParse(c6, &telemetry6, &recStatus6);
	  if (x6 == PARSE_OK){
		  successESC6++;
		  TelemetryArray[6] = telemetry6;
	  }
	  else if (x6 == PARSE_ERROR){
		  failESC6++;
	  }
  }
}
/* ESC 0 && ESC 7 ---------------------------------------------------------*/
void Parse_ESC1(void const *parameter){
	while (1){
		aRxBuffer_0 = getdata(0);
		aRxBuffer_7 = getdata(1);
		RINGBUF_Put((RINGBUF*) ringbuf0, aRxBuffer_0);
		RINGBUF_Put((RINGBUF*) ringbuf7, aRxBuffer_7);
		RINGBUF_Get((RINGBUF*) ringbuf0, &c0);
		RINGBUF_Get((RINGBUF*) ringbuf7, &c7);
		int x0 = telemetryParse(c0, &telemetry0, &recStatus0);
		if (x0 == PARSE_OK){
			successESC0++;
			TelemetryArray[0] = telemetry0;
		}
		else if (x0 == PARSE_ERROR){
			failESC0++;
		}
		int x7 = telemetryParse(c7, &telemetry7, &recStatus7);
		if (x7 == PARSE_OK){
			successESC7++;
			TelemetryArray[7] = telemetry7;
		}
		else if (x7 == PARSE_ERROR){
			failESC7++;
		}
	}
}
void Parse_ESC2(void const *parameter){
	while (1){
		  transmit_mavlink_esc_1_to_4(esc1);
		  osDelay(10);
		  transmit_mavlink_esc_5_to_8(esc2);
		  osDelay(10);
	}
}
void Parse_ESC3(void const *parameter){
	while (1){
//		aRxBuffer_5 = getdata(2);
//		int x5 = telemetryParse(c5, &telemetry5, &recStatus5);
//		if (x5 == PARSE_OK){
//			successESC5++;
//			TelemetryArray[5] = telemetry5;
//		}
//		else if (x5 == PARSE_ERROR){
//			failESC5++;
//		}
	}
}
void Parse_ESC4(void const *parameter){
	while (1){}
}
void Parse_ESC5(void const *parameter){
	while (1){}
}
void Parse_ESC6(void const *parameter){
	while (1){}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM10)
	{
		SoftUartHandler();
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
