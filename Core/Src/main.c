/* USER CODE BEGIN Header */
/**
****************************************************************************
**
* @file : main.c
* @brief : Main program body
*
* UART2 OPTIMIZATION IMPLEMENTATION:
* ==================================
* - UART2 hardware remains initialized for future compatibility
* - Software features (debug, manual input, status) are conditionally compiled
* - Performance mode: All UART2 features disabled (macros set to 0)
* - UART1 (camera) always enabled for real-time ball tracking
*
* Benefits: Reduced interrupt overhead, faster main loop, less memory usage
****************************************************************************
**
* @attention
*
* Copyright (c) 2025 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
****************************************************************************
**
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "ball_and_plate.h"
#include "ball_position.h"
// #include "pid_controller.h" // Classic PID is no longer used
#include "uart_handler.h"
#include "pca9685_servo.h"
#include "fuzzy_sugeno_pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#endif

// ============================================================================
// UART2 FUNCTIONALITY CONTROL
// ============================================================================
// Set to 1 to enable UART2 features, 0 to disable for performance
#define UART2_MANUAL_INPUT      0   // Manual coordinate input via UART2  
#define UART2_DEBUG_OUTPUT      0   // Debug output via UART2 - DISABLED to avoid bugs

// ============================================================================
// PURE PID CONTROL SYSTEM
// ============================================================================
// Simple ball balancing system using PID control:
// 1. Camera data (UART1) provides ball position
// 2. PID controller calculates tilt/phi angles
// 3. Stewart platform moves to balance ball
// 4. UART2 used for debug output and status monitoring
// ============================================================================

// ============================================================================
// UART2 HELPER MACROS - Conditional compilation for performance
// ============================================================================
#if UART2_DEBUG_OUTPUT  
    #define UART2_DEBUG_PRINT(msg) UartTransmitString(&uartHandler, msg)
#else
    #define UART2_DEBUG_PRINT(msg) // Do nothing - optimized out
#endif

#if UART2_MANUAL_INPUT
    #define UART2_PROCESS_INPUT() UartProcessData(&uartHandler, &ball)
#else
    #define UART2_PROCESS_INPUT() false // Return false - no manual input
#endif
// ============================================================================

// LED definitions - removed for cleaner code
//#define SERVO_UPDATE_PERIOD 20  //
#define STATUS_UPDATE_PERIOD 2000 // 2000ms update period for status messages
#define BALL_TIMEOUT_PERIOD 2500 // Moderate timeout for stability

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// Robot and control variables
RobotState robot;
// PIDController pid;        // Classic PID - REMOVED
FuzzySugenoPID fuzzy_pid; // Fuzzy PID
BallPosition ball;
UartHandler uart1Handler;

// Camera data
CameraData cameraData;

// Target data
TargetData targetData;

// Timing variables
uint32_t lastCameraCheckTime = 0;
uint32_t ball_lost_timestamp = 0;
uint32_t lastPidUpdateTime = 0;

// The `use_fuzzy_pid` flag has been removed. The system now exclusively uses the Fuzzy PID controller.
// The internal fuzzy logic can still be toggled using `FuzzyPIDEnableFuzzy()`.
// bool use_fuzzy_pid = true;  // ← CHỈ THAY ĐỔI true/false ĐỂ SWITCH CONTROLLER
// ============================================================================
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
/* System startup complete */

/* Initialize system components */
// Initialize PCA9685 for servo control using new library
PCA9685_Init(50); // 50Hz frequency for servos

// Test I2C connectivity
if (PCA9685_TestConnection()) {
    } else {
}

// Initialize the robot
RobotInit(&robot);
FuzzyPIDInit(&fuzzy_pid, 0.56f, 0.13f, 0.17f, 1.5f, 1.0f, 0.033f, 0);
//////////////60FPS
//1.235f, 0.3f, 0.255f, 1.5f, 1.0f, 0.029f, 0
//0.74f, 0.008f, 0.152f, 1.45f, 1.0f, 0.085f, 0
//0.53f, 0.08f, 0.17f, 1.5f, 1.0f, 0.08f, 0
//1.21f, 0.0065f, 0.27f, 1.5f, 1.0f, 0.016f, 0
///////////////////////////////////////////////////////////////////////////
//////////////30Fps
//1.6f, 0.03f, 0.3f, 1.55f, 1.0f, 0.1f, 1
//1.28f, 0.006f, 0.343f, 1.4f, 0.95f, 0.095f, 0 tanh=2.5 good
//1.591f, 0.0033f, 0.431f, 1.5f, 1.0f, 0.06f, 0 tanh=2.5 good
//0.76f, 0.005f, 0.327f, 1.55f, 1.0f, 0.15f, 0 tanh = 2.5 mode square, circle good
//1.95f, 0.0f, 0.6f, 1.55f, 1.0f, 0.1f, 1
///////////////////////////////////////////////////////////////////////////
//0.17f, 0.001f, 0.24f, 2.5f, 1.0f, 0.15f, 0 matlab
//1.28f, 0.05f, 0.343f, 1.55f, 0.07f, 0
//1.58f, 0.00036f, 0.42f, 1.45f, 0.06f, 0 ổn
//1.305f, 0.1f, 0.343f, 2.5f, 1.55f, 0.07f, 0  rung mạnh
//1.31f, 0.3f, 0.34f, 1.55f, 0.07f, 0
//1.32f, 0.3f, 0.33f, 2.5f, 1.5f, 0.06f, 0
//1.75f, 0.4f, 0.36f, 2.5f, 1.55f, 0.05f, 0);
//1.83f, 0.44f, 0.41f, 2.5f, 1.55f, 0.05f, 0
//1.95f, 0.65f, 0.62f, 2.5f, 1.55f, 0.08f, 1 chạy được
//2.09f, 0.49f, 0.53f, 2.5f, 1.6f, 0.08f, 1 chạy được
///////////////////////////////////////////////////////////////////////////
FuzzyPIDEnableFuzzy(&fuzzy_pid, false);
FuzzyPIDSetTarget(&fuzzy_pid, 0.0f, 0.0f);  // Initialize with center target
// E_max, D_max, K_e, K_d_in, A_PD, A_I, U_mid, U_max
FuzzyPIDInitFuzzy(&fuzzy_pid, 9.0f, 15.0f, 0.9f, 0.9f, 0.03f, 0.001f, 0.35f, 0.7f);
//9.0f, 15.0f, 0.9f, 0.9f, 0.08f, 0.001f, 0.35f, 0.7f mode square good
//8.0f, 40.0f, 0.25f, 0.12f, 0.085f, 0.4f, 0.08f, 0.15f 
//9.0f, 15.0f, 0.9f, 0.9f, 0.06f, 0.001f, 0.2f, 0.4f   Kp=1.28

FuzzyPIDSetDerivativeFilter(&fuzzy_pid, 1.0f);  // Fuzzy PID derivative filter
// End fuzzy PID----------------------

BallPositionInit(&ball);

// Initialize target data
memset(&targetData, 0, sizeof(TargetData));
targetData.x = 0.0f;
targetData.y = 0.0f;
targetData.valid = true;
targetData.updated = false;

// Initialize UART with DMA

// UART2 disabled to avoid PSO-related compilation errors
// UartInit(&uartHandler, &huart2, &hdma_usart2_rx);      // Debug → UART2
// UartStartReceive(&uartHandler);

/* Initialize UART1 for camera data */
UartInit(&uart1Handler, &huart1, &hdma_usart1_rx);     // Camera → UART1
UartStartReceive(&uart1Handler);

/* Initial delay to ensure all systems are ready */
HAL_Delay(500);

/* Initialize servos to level position at current height */
PerformInitialMovements(&robot);

/* Main loop variables */
uint32_t currentTime;
float tilt = 0.0f, phi = 0.0f;  // Initialize PID output variables
bool plate_reset_initiated = false; // Flag to track if plate reset has been initiated

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Update servo motion if in progress - MUST BE CALLED EVERY LOOP */
    UpdateServoMotion();

    /* Get current time */
    currentTime = HAL_GetTick();

    /* Check DMA buffer for camera UART only */
    UartCheckDMA(&uart1Handler);  // UART1 - Camera - Always check

    /* Check for new target data and update setpoint if received */
    if (UartGetLatestTargetData(&uart1Handler, &targetData)) {
        if (targetData.valid && targetData.updated) {
            // Update setpoint for the fuzzy controller
            FuzzyPIDSetTarget(&fuzzy_pid, targetData.x, targetData.y);
        }
    }

    /* Check camera timeout every 500ms */
    if (currentTime - lastCameraCheckTime >= 500) {
        UartCameraTimeout(&uart1Handler, 1000); // 1 second timeout
        lastCameraCheckTime = currentTime;
    }

    /* Try to get camera data from UART1 */
    bool cameraDataAvailable = UartGetLatestCameraData(&uart1Handler, &cameraData);
    
    /* Camera data processing and ball loss detection */
    if (cameraDataAvailable) {
        // Ball is detected: update its position and reset the loss timer.
        BallPositionUpdate(&ball, cameraData.x, cameraData.y);
        ball_lost_timestamp = 0;
        plate_reset_initiated = false; // Reset the plate reset flag
    } else {
        // No new camera data: check if the ball was previously detected.
        if (ball.isDetected) {
            if (ball_lost_timestamp == 0) {
                // This is the first frame without data; start the timer.
                ball_lost_timestamp = currentTime;
            } else if (currentTime - ball_lost_timestamp > 1000 && !plate_reset_initiated) {
                // More than 1 second has passed since the ball was lost.
                ball.isDetected = false; // Officially declare the ball as lost.
                // Reset the fuzzy controller
                FuzzyPIDReset(&fuzzy_pid);
                // Smoothly return the plate to the center.
                StartMoveTo(&robot, 0.0f, 0.0f, robot.geom.h, 1000);
                plate_reset_initiated = true; // Mark that reset has been initiated
            }
        }
    }

    /* Update PID control ONLY if the ball is currently detected AND not in loss grace period AND plate is not resetting */
    if (ball.isDetected && ball_lost_timestamp == 0 && !plate_reset_initiated) {
        // Only update PID when we have fresh data (not in the 1s grace period) and not returning to center
        lastPidUpdateTime = currentTime;
        
        // === USE FUZZY PID CONTROL ===
        tilt = FuzzyPIDUpdateWithTime(&fuzzy_pid, ball.x, ball.y);
        phi = fuzzy_pid.output_phi;
        
        // Move platform immediately based on PID output.
        StartMoveTo(&robot, tilt, phi, robot.geom.h, 0);
    }

    /* Ultra-minimal delay - yield to interrupts only */
    HAL_Delay(0); // No fixed delay, just yield CPU for interrupts
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
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
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 115200;
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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

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
    // System error - infinite loop
    HAL_Delay(100);
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
/* Print error message */
// char errorMsg[100];
// sprintf(errorMsg, "Assert Failed: file %s on line %ld\r\n", (char*)file, line);
// UART2_STATUS_PRINT(errorMsg);
/* Enter infinite loop */
Error_Handler();
/* User can add his own implementation to report the file name and line number,
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
