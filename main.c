/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _PIN {
	GPIO_TypeDef* port;
	uint16_t pin;
} PIN;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SDADC_HandleTypeDef hsdadc1;
SDADC_HandleTypeDef hsdadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

// CANBUS Messages
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
uint32_t              TxMailbox;

// Torque
CAN_TxHeaderTypeDef   TxHeaderTorque;
uint8_t               TxDataTorque[8];


// Throttle
CAN_TxHeaderTypeDef   TxHeaderThrottle;
uint8_t               TxDataThrottle[8];


// Brake
CAN_TxHeaderTypeDef   TxHeaderBrake;
uint8_t               TxDataBrake[8];


// Switch
CAN_TxHeaderTypeDef   TxHeaderSwitch;
uint8_t               TxDataSwitch[8];


// State
CAN_TxHeaderTypeDef   TxHeaderState;
uint8_t               TxDataState[8];



// CANBUS
const uint32_t CANID_PDM_0 = 0x520;
const uint32_t CANID_PDM_1 = 0x521;
const uint32_t CANID_PDM_2 = 0x522;
const uint32_t CANID_PDM_3 = 0x524;

const uint32_t ADC_RESOLUTION = 16;
const float SENSOR_MIN = 0.1;
const float SENSOR_MAX = 0.9;

uint8_t print_throttle = 0;
uint8_t print_brake = 0;
uint8_t print_canbus = 0;

enum CarStates
{
  IDLE,
  PRECHARGED,
  RTD,
  FAULT
} carState;


// Start Button
const PIN START_BUTTON_PIN = {GPIOB, GPIO_PIN_8};
uint8_t start_button_press = 1;

// Fault LEDs
struct Faults
{
	uint8_t ams;
	uint8_t pdoc;
	uint8_t imd;
	uint8_t bspd;
} faults;

const PIN AMS_FAULT_LED = {GPIOC, GPIO_PIN_14};
const PIN IMD_FAULT_LED = {GPIOF, GPIO_PIN_9};
const PIN PDOC_FAULT_LED = {GPIOC, GPIO_PIN_15};
const PIN BSPD_FAULT_LED = {GPIOE, GPIO_PIN_6};


// APPS & Brake Plausibility Checks
struct AppsBrakeErrors
{
	uint8_t appsBrakePlausibility;
	uint8_t tpsRange;
	uint8_t brakeRange;
	uint8_t apps;
	uint8_t appsBrake;
} apps_brake_errors;



// PDM State
struct PDM_Status
{
	uint8_t precharge;
	uint8_t precharged;
	uint8_t rtd;
	uint8_t drive_enable;
} pdm_status;


// ADC Raw
struct ADC_Raw
{
  uint16_t tps1;
  uint16_t tps2;
  uint16_t front;
  uint16_t rear;
} adc_raw;

uint8_t adc_avg_filter = 4;

// Throttle Configuration


struct Throttle
{
  uint16_t tps1;
  uint16_t tps2;
  uint16_t pos;
} throttle;

const uint32_t TPS1_MIN = 4300;
const uint32_t TPS1_MAX= 26000;

const uint32_t TPS2_MIN = 4000;
const uint32_t TPS2_MAX= 30000;

const uint32_t TORQUE_MIN = 0;
const uint32_t TORQUE_MAX = 300;
//const uint32_t TORQUE_MAX = 2400;


// Brake Configuration


struct Brake
{
  uint16_t front;
  uint16_t rear;
  uint16_t avg;
} brake;

const uint32_t BRAKE_FRONT_MIN = 7000;
const uint32_t BRAKE_FRONT_MAX = 58900;

const uint32_t BRAKE_REAR_MIN = 7000;
const uint32_t BRAKE_REAR_MAX = 58900;

const uint32_t BRAKE_PRESSURE_MIN = 0;
const uint32_t BRAKE_PRESSURE_MAX = 13789;

const uint32_t BRAKE_PRESSURE_THRESHOLD = 1000; // For checkAppsBrakePlausibilityError()

// CAN message frequency
const uint32_t TORQUE_SEND_FREQ = 5000;
const uint32_t FAULT_CHECK_FREQ = 5000;


// Mapped values
int mc_torque = 0;
int throttle_pos = 0;
int brake_avg = 0;


//uint16_t ADC_Throttle[2];
//uint16_t ADC_Brake[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SDADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SDADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void readCAN(void);
int getTorqueCommand(void);
void sendTorqueCommand(void);
void sendThrottlePosition(void);
void sendBrakePressure(void);
void sendStartSw(void);
void sendState(void);
void updateLEDs(void);
uint8_t checkFault(void);
void checkAppsBrakePlausibilityError(void);




float constrain(float val, float lower, float upper);
float map(float val, float fromLow, float fromHigh, float toLow, float toHigh);

void print_ADC(void);
void print_CANBUS_RxData(void);

int _write(int file, char *ptr, int len){

	for(int i=0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

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
  carState = IDLE;

  // Torque
  TxHeaderTorque.StdId = 0xC0;
  TxHeaderTorque.RTR = CAN_RTR_DATA;
  TxHeaderTorque.IDE = CAN_ID_STD;
  TxHeaderTorque.DLC = 8;

  // Throttle
  TxHeaderThrottle.StdId = 0x244;
  TxHeaderThrottle.RTR = CAN_RTR_DATA;
  TxHeaderThrottle.IDE = CAN_ID_STD;
  TxHeaderThrottle.DLC = 3;

  // Brake
  TxHeaderBrake.StdId = 0x245;
  TxHeaderBrake.RTR = CAN_RTR_DATA;
  TxHeaderBrake.IDE = CAN_ID_STD;
  TxHeaderBrake.DLC = 6;

  // Switch
  TxHeaderSwitch.StdId = 0x650;
  TxHeaderSwitch.RTR = CAN_RTR_DATA;
  TxHeaderSwitch.IDE = CAN_ID_STD;
  TxHeaderSwitch.DLC = 2;

  // State
  TxHeaderState.StdId = 0x651;
  TxHeaderState.RTR = CAN_RTR_DATA;
  TxHeaderState.IDE = CAN_ID_STD;
  TxHeaderState.DLC = 6;


  adc_raw.tps1 = 0;
  adc_raw.tps2 = 0;
  adc_raw.front = 0;
  adc_raw.rear = 0;

  throttle.tps1 = 0;
  throttle.tps2 = 0;
  throttle.pos = 0;

  brake.front = 0;
  brake.rear = 0;
  brake.avg = 0;

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
  MX_CAN_Init();
  MX_SDADC1_Init();
  MX_TIM3_Init();
  MX_SDADC2_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	switch (carState)
	{
		// Idle
		case IDLE:
			updateLEDs();
			// NextState = Fault
			if(checkFault()) {
				carState = FAULT;
				printf("\tNext State: FAULT\n");
			}

			// If precharge complete
			else if (pdm_status.precharged) {
				// NextState = Precharged
				carState = PRECHARGED;
				printf("\tNext State: PRECHARGED\n");
			}
			break;


		// Precharged
		case PRECHARGED:

			updateLEDs();
			// NextState = Fault
			if(checkFault()) {
				carState = FAULT;
				printf("\tNext State: FAULT\n");
			}

			// Poll Start Button
			else if (!HAL_GPIO_ReadPin(START_BUTTON_PIN.port, START_BUTTON_PIN.pin) && brake.avg > BRAKE_PRESSURE_THRESHOLD)
//			else if (!HAL_GPIO_ReadPin(START_BUTTON_PIN.port, START_BUTTON_PIN.pin))
			{
				sendStartSw();
				// NextState = RTD
				carState = RTD;
				printf("\tNext State: RTD\n");
			}
			break;


		// Ready To Drive
		case RTD:
			// NextState = Fault
			if(checkFault()) {
				carState = FAULT;
				printf("\tNext State: FAULT\n");
			}

			// Send Torque to MC
			else{
			}
			break;


		// Fault
		case FAULT:
			updateLEDs();
			if(!checkFault()) {
				carState = IDLE;
				printf("\tNext State: IDLE\n");
			}
			break;

	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDADC;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG2);
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  uint32_t id1 = (CANID_PDM_0 << 21);
  uint32_t id2 = (CANID_PDM_2 << 21);

   /* Configure the CAN Filter */
   sFilterConfig.FilterBank = 0;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterIdHigh = (id1 >> 16);
   sFilterConfig.FilterIdLow = (id1 & 0xFFFF);
   sFilterConfig.FilterMaskIdHigh = (id2 >> 16);
   sFilterConfig.FilterMaskIdLow = (id2 & 0xFFFF);
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 13;


   if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
   {
     /* Filter configuration Error */
     Error_Handler();
   }

   /* Start the CAN peripheral */
   if (HAL_CAN_Start(&hcan) != HAL_OK)
   {
     /* Start Error */
     Error_Handler();
   }

   /* Activate CAN RX notification */
   if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   {
     /* Notification Error */
     Error_Handler();
   }


//  TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SDADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDADC1_Init(void)
{

  /* USER CODE BEGIN SDADC1_Init 0 */

  /* USER CODE END SDADC1_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC1_Init 1 */

  /* USER CODE END SDADC1_Init 1 */

  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc1.Instance = SDADC1;
  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_EXT;
  hsdadc1.InjectedTrigger = SDADC_SOFTWARE_TRIGGER;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Mode
  */
  if (HAL_SDADC_SelectInjectedDelay(&hsdadc1, SDADC_INJECTED_DELAY_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_InjectedConfigChannel(&hsdadc1, SDADC_CHANNEL_2|SDADC_CHANNEL_1, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_2, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_1, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC1_Init 2 */

  /* Start Calibration in polling mode */
//  if (HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1) != HAL_OK)
//  {
//    /* An error occurs during the starting phase of the calibration */
//	  Error_Handler();
//  }
//
//  /* Pool for the end of calibration */
//  if (HAL_SDADC_PollForCalibEvent(&hsdadc1, 1000) != HAL_OK)
//  {
//    /* An error occurs while waiting for the end of the calibration */
//	  Error_Handler();
//  }

  /* USER CODE END SDADC1_Init 2 */

}

/**
  * @brief SDADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDADC2_Init(void)
{

  /* USER CODE BEGIN SDADC2_Init 0 */

  /* USER CODE END SDADC2_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC2_Init 1 */

  /* USER CODE END SDADC2_Init 1 */

  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc2.Instance = SDADC2;
  hsdadc2.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc2.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc2.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc2.Init.ReferenceVoltage = SDADC_VREF_EXT;
  hsdadc2.InjectedTrigger = SDADC_SOFTWARE_TRIGGER;
  if (HAL_SDADC_Init(&hsdadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Mode
  */
  if (HAL_SDADC_SelectInjectedDelay(&hsdadc2, SDADC_INJECTED_DELAY_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedTrigger(&hsdadc2, SDADC_SOFTWARE_TRIGGER) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_InjectedConfigChannel(&hsdadc2, SDADC_CHANNEL_3|SDADC_CHANNEL_2, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc2, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_3, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_2, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC2_Init 2 */

//  if (HAL_SDADC_CalibrationStart(&hsdadc2, SDADC_CALIBRATION_SEQ_1) != HAL_OK)
//  {
//    /* An error occurs during the starting phase of the calibration */
//	  Error_Handler();
//  }
//
//  /* Pool for the end of calibration */
//  if (HAL_SDADC_PollForCalibEvent(&hsdadc2, 1000) != HAL_OK)
//  {
//    /* An error occurs while waiting for the end of the calibration */
//	  Error_Handler();
//  }

  /* USER CODE END SDADC2_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 7999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 500;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sendTorqueCommand(void)
{
	// Calculate Torque Command
	uint16_t torque = map(throttle.pos, 0, 100, TORQUE_MIN, TORQUE_MAX);


	// Transmit Torque
	TxDataTorque[0] = (uint8_t)(torque & 0xFF); // Low Byte
	TxDataTorque[1] = (uint8_t)((torque >> 8) & 0xFF); // High Byte
	TxDataTorque[2] = 0;
	TxDataTorque[3] = 0;

	if (apps_brake_errors.appsBrakePlausibility) {
		TxDataTorque[4] = 0; // Disable inverter
	}
	else{
		TxDataTorque[4] = 1; // Enable inverter
	}

	TxDataTorque[5] = 1; //Forward mode
	TxDataTorque[6] = 0;
	TxDataTorque[7] = 0;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderTorque, TxDataTorque, &TxMailbox) != HAL_OK)
	{
		printf("Send Throttle\n");
	  /* Transmission request Error */
	  Error_Handler();
	}
}


void sendThrottlePosition(void)
{
	// Transmit Torque
	TxDataThrottle[0] = (uint8_t)(throttle.pos & 0xff);
	TxDataThrottle[1] = (uint8_t)(throttle.tps1 & 0xff);
	TxDataThrottle[2] = (uint8_t)(throttle.tps2 & 0xff);
	TxDataThrottle[3] = 0;
	TxDataThrottle[4] = 0;
	TxDataThrottle[5] = 0;
	TxDataThrottle[6] = 0;
	TxDataThrottle[7] = 0;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderThrottle, TxDataThrottle, &TxMailbox) != HAL_OK)
	{
		printf("Send Brake Pressure\n");
	  /* Transmission request Error */
	  Error_Handler();
	}
}

void sendBrakePressure(void)
{
	// Transmit Torque
	TxHeaderBrake.StdId = 0x245;
	TxHeaderBrake.RTR = CAN_RTR_DATA;
	TxHeaderBrake.IDE = CAN_ID_STD;
	TxHeaderBrake.DLC = 6;

	TxDataBrake[0] = ((brake.avg >> 8) & 0xff);
	TxDataBrake[1] = ((brake.avg ) & 0xff);
	TxDataBrake[2] = ((brake.rear >> 8) & 0xff);
	TxDataBrake[3] = ((brake.rear) & 0xff);
	TxDataBrake[4] = ((brake.front  >> 8) & 0xff);
	TxDataBrake[5] = ((brake.front) & 0xff);
	TxDataBrake[6] = 0;
	TxDataBrake[7] = 0;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderBrake, TxDataBrake, &TxMailbox) != HAL_OK)
	{
		printf("Send Brake Pressure\n");
	  /* Transmission request Error */
	  Error_Handler();
	}
}


void sendStartSw(void)
{
	TxDataSwitch[0] = 1; // Low Byte

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderSwitch, TxDataSwitch, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  Error_Handler();
	}
}

void sendState(void)
{
	TxDataState[0] = (uint8_t) carState;
	TxDataState[1] = apps_brake_errors.appsBrakePlausibility;
	TxDataState[2] = apps_brake_errors.tpsRange;
	TxDataState[3] = apps_brake_errors.brakeRange;
	TxDataState[4] = apps_brake_errors.apps;
	TxDataState[5] = apps_brake_errors.appsBrake;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeaderState, TxDataState, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  Error_Handler();
	}
}

void updateLEDs(void)
{
	if (faults.ams) {
		HAL_GPIO_WritePin(AMS_FAULT_LED.port, AMS_FAULT_LED.pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(AMS_FAULT_LED.port, AMS_FAULT_LED.pin, GPIO_PIN_RESET);
	}

	if (faults.imd) {
		HAL_GPIO_WritePin(IMD_FAULT_LED.port, IMD_FAULT_LED.pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(IMD_FAULT_LED.port, IMD_FAULT_LED.pin, GPIO_PIN_RESET);
	}

	if (faults.pdoc) {
		HAL_GPIO_WritePin(PDOC_FAULT_LED.port, PDOC_FAULT_LED.pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(PDOC_FAULT_LED.port, PDOC_FAULT_LED.pin, GPIO_PIN_RESET);
	}

	if (faults.bspd) {
		HAL_GPIO_WritePin(BSPD_FAULT_LED.port, BSPD_FAULT_LED.pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(BSPD_FAULT_LED.port, BSPD_FAULT_LED.pin, GPIO_PIN_RESET);
	}
}

uint8_t checkFault(void)
{
	if(faults.ams || faults.pdoc || faults.imd || faults.bspd){
		return 1;
	}
	else {
		return 0;
	}
}

void checkAppsBrakePlausibilityError(void)
{


	uint16_t adc_min = SENSOR_MIN * pow(2, ADC_RESOLUTION);
	uint16_t adc_max = SENSOR_MAX * pow(2, ADC_RESOLUTION);

	// RANGE CHECK
	if (adc_raw.tps1 < adc_min || adc_raw.tps1 > adc_max) {
		apps_brake_errors.tpsRange = 1;
	}
	else if (adc_raw.tps2 < adc_min || adc_raw.tps2 > adc_max) {
		apps_brake_errors.tpsRange = 1;
	}
	else {
		apps_brake_errors.tpsRange = 0;
	}


	if (adc_raw.front < adc_min || adc_raw.front > adc_max) {
		apps_brake_errors.brakeRange = 1;
	}
	else if (adc_raw.rear < adc_min || adc_raw.rear > adc_max) {
		apps_brake_errors.brakeRange = 1;
	}
	else {
		apps_brake_errors.brakeRange = 0;
	}




	// CHECK APPS
	if (abs(throttle.tps1 - throttle.tps2) > 10) {
		apps_brake_errors.apps = 1;
	}
	else {
		apps_brake_errors.apps = 0;
	}


	// CHECK APPS BRK
	if (brake.avg > BRAKE_PRESSURE_THRESHOLD && throttle.pos > 25) {
		apps_brake_errors.appsBrake = 1;
	}

	// CLEAR APPS BRAKE PLAUSIBILITY IF THROTTLE LESS THAN 5%
	if (throttle.pos < 5){
		apps_brake_errors.appsBrake = 0;
	}

	apps_brake_errors.appsBrakePlausibility =
			(apps_brake_errors.tpsRange ||
			apps_brake_errors.brakeRange ||
			apps_brake_errors.apps ||
			apps_brake_errors.appsBrake);

//	apps_brake_errors.appsBrakePlausibility = 0;
}


float constrain(float val, float lower, float upper)
{
	float newVal = val;
	if(val < lower) newVal = lower;
	if (val > upper) newVal = upper;

	return newVal;
}

float map(float val, float fromLow, float fromHigh, float toLow, float toHigh)
{
	return (val - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
	printf("HAL_CAN_GetRxMessage()\n");
    Error_Handler();
  }

  // PDM Message0
  if (RxHeader.StdId == CANID_PDM_0) {
	  faults.ams = RxData[0];
	  faults.pdoc = RxData[1];
	  faults.bspd = RxData[2];
	  faults.imd = RxData[3];
	  pdm_status.precharge = RxData[4];
	  pdm_status.precharged = RxData[5];
	  pdm_status.rtd = RxData[6];
	  pdm_status.drive_enable = RxData[7];
  }

//  if (RxHeader.StdId == CANID_PDMMESSAGE0) {
////	  memcpy(&faults, RxData, sizeof(faults));
//	  memcpy(&pdm_status, RxData+4, sizeof(pdm_status));
//  }

  if (print_canbus){
	  print_CANBUS_RxData();
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    switch((uint32_t)htim->Instance)    {

        case (uint32_t)TIM2:

			uint32_t channel1;
        	uint32_t channel2;
        	uint32_t channel3;
        	uint32_t channel4;
        	uint16_t adc_val;

        	// Read Throttle
        	HAL_SDADC_InjectedStart(&hsdadc2);
        	HAL_SDADC_PollForInjectedConversion(&hsdadc2, 1000);
        	adc_val = HAL_SDADC_InjectedGetValue(&hsdadc2, &channel1) - 32770;
			adc_raw.tps1 = (adc_raw.tps1 * (adc_avg_filter - 1) + adc_val) / adc_avg_filter;;

        	HAL_SDADC_InjectedStart(&hsdadc2);
        	HAL_SDADC_PollForInjectedConversion(&hsdadc2, 1000);
        	adc_val = HAL_SDADC_InjectedGetValue(&hsdadc2, &channel2) - 32770;
			adc_raw.tps2 = (adc_raw.tps2 * (adc_avg_filter - 1) + adc_val) / adc_avg_filter;;


			adc_val = constrain(adc_raw.tps1, TPS1_MIN, TPS1_MAX);
        	throttle.tps1 = map(adc_val, TPS1_MAX, TPS1_MIN, 0, 100);

        	adc_val = constrain(adc_raw.tps2, TPS2_MIN, TPS2_MAX);
        	throttle.tps2 = map(adc_val, TPS2_MAX, TPS2_MIN, 0, 100);

        	throttle.pos = (throttle.tps1 + throttle.tps2) / 2;


			// Read Brakes
			HAL_SDADC_InjectedStart(&hsdadc1);
			HAL_SDADC_PollForInjectedConversion(&hsdadc1, 1000);
			adc_val = HAL_SDADC_InjectedGetValue(&hsdadc1, &channel3) - 32770;
        	adc_raw.front = (adc_raw.front * (adc_avg_filter - 1) + adc_val) / adc_avg_filter;;

			HAL_SDADC_InjectedStart(&hsdadc1);
			HAL_SDADC_PollForInjectedConversion(&hsdadc1, 1000);
			adc_val = HAL_SDADC_InjectedGetValue(&hsdadc1, &channel4) - 32770;
        	adc_raw.rear = (adc_raw.rear * (adc_avg_filter - 1) + adc_val) / adc_avg_filter;

        	// CONVERT TO PRESSURE
        	adc_val = constrain(adc_raw.front, BRAKE_FRONT_MIN, BRAKE_FRONT_MAX);
        	brake.front = map(adc_val, BRAKE_FRONT_MIN, BRAKE_FRONT_MAX, BRAKE_PRESSURE_MIN, BRAKE_PRESSURE_MAX);

        	adc_val = constrain(adc_raw.rear, BRAKE_REAR_MIN, BRAKE_REAR_MAX);
        	brake.rear = map(adc_val, BRAKE_REAR_MIN, BRAKE_REAR_MAX, BRAKE_PRESSURE_MIN, BRAKE_PRESSURE_MAX);

        	brake.avg = (brake.front + brake.rear) / 2;

            break;


        case (uint32_t)TIM4:
			// If Ready To Drive then send Torque
			if (carState == RTD){

				sendTorqueCommand();
			}

        	checkAppsBrakePlausibilityError();
			sendThrottlePosition();
			sendBrakePressure();


			break;


        case (uint32_t)TIM5:
			print_ADC();
        	sendState();
//        	printf("Precharged: [%d]\n", pdm_status.precharged);
			break;
    }
}


void print_ADC(void)
{

	if (print_throttle){
		printf("TPS1: [%d],", adc_raw.tps1);
		printf("TPS2: [%d]", adc_raw.tps2);
	}

	if (print_brake){
		printf("\tBrakeF: [%d],", adc_raw.front);
		printf("BrakeR: [%d]", adc_raw.rear);
	}

	if (print_throttle || print_brake){
		printf("\n");
	}

}

void print_CANBUS_RxData(void) {

	printf("[%d] -  ", (int) RxHeader.StdId);
	printf("RxData: ");
	for (int i = 0; i < RxHeader.DLC; i++){
		printf(" %d,", RxData[i]);
	}
	printf("\n");
}
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
