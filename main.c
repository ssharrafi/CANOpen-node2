/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include        "CANopen.h"
#include        "CO_OD.h"
#include        "CO_config.h"
#include        "CO_SDOclient.h"
#include        "CO_app_STM32.h"
#include       "CO_driver.h"

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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

CANopenNodeSTM32 canOpenNodeSTM32;

extern CO_t* CO;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 //Function for SDO download(read from driver)
  CO_SDO_return_t performSDODOWNLOAD(CO_SDOclient_t* SDO_C, uint16_t index, uint8_t subIndex, void* data, size_t size)
 {
  
    CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    size_t sizeIndicated;
    size_t sizeTransferred;
    //(read in driver)
    CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, size, 1000, false);
    CO_SDOclientDownloadBufWrite(SDO_C,(uint8_t*) data, size);
    while (CO_SDOclientDownload(SDO_C, 1000, false, false, &abortCode, &sizeIndicated, &sizeTransferred) == CO_SDO_RT_waitingResponse)
    {
    HAL_Delay(1);
    }
    return (abortCode != CO_SDO_AB_NONE) ? CO_SDO_RT_endedWithClientAbort : CO_SDO_RT_ok_communicationEnd;
   
  }
 
  //function for SDO upload(write in driver)
  CO_SDO_return_t performSDOUpload(CO_SDOclient_t* SDO_C, uint16_t index, uint8_t subIndex, void* data, size_t size) {
    CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    size_t sizeIndicated;
    size_t sizeTransferred;
    uint32_t timerNext_us;

    CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
    while (CO_SDOclientUpload(SDO_C, 1000, false, &abortCode,&sizeIndicated, &sizeTransferred, &timerNext_us) == CO_SDO_RT_waitingResponse) {
        HAL_Delay(1);
    }
    if (abortCode != CO_SDO_AB_NONE) {
        return CO_SDO_RT_endedWithServerAbort;
    }
    size_t readSize = CO_SDOclientUploadBufRead(SDO_C, (uint8_t*)data, size);
    return (readSize != size) ? CO_SDO_RT_wrongArguments : CO_SDO_RT_ok_communicationEnd;
}
 
 //function for motor position(send position to driver)
CO_SDO_return_t setMotorPosition(CO_SDOclient_t* SDO_c, int32_t position){
CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
    size_t sizeIndicated;
    size_t sizeTransferred;

    CO_SDOclientDownloadInitiate(SDO_c,0x607A, 0,sizeof(position), 1000, false);
    CO_SDOclientDownloadBufWrite(SDO_c,(uint8_t*)&position,sizeof(position));
    while (CO_SDOclientDownload(SDO_c, 1000,false, false, &abortCode,&sizeIndicated, &sizeTransferred) == CO_SDO_RT_waitingResponse) {
        HAL_Delay(1);
    }
    
    return (abortCode != CO_SDO_AB_NONE) ? CO_SDO_RT_wrongArguments : CO_SDO_RT_ok_communicationEnd;
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
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    canOpenNodeSTM32.CANHandle = &hcan;
    canOpenNodeSTM32.HWInitFunction = MX_CAN_Init;
    canOpenNodeSTM32.timerHandle = &htim2; // ???: TIM2
    canOpenNodeSTM32.desiredNodeID = 1; // Node-ID = 1
    canOpenNodeSTM32.baudrate = 500; // 500 kbps

    CO = CO_new(NULL, NULL);
    if (CO == NULL) Error_Handler();

    CO_ReturnError_t err = CO_CANinit(CO ,&canOpenNodeSTM32,0);
    if (err != CO_ERROR_NO) Error_Handler();
    
 //Homming Mode driver NODE ID=2
   
    int8_t mode=6;
    if (performSDODOWNLOAD(CO->SDOclient,0x6060,0,&mode,sizeof(mode)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
 
    int8_t hommingMethod = 35; //homming methode is current position
    if (performSDODOWNLOAD(CO->SDOclient,0x6098,0,&hommingMethod,sizeof(hommingMethod)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    uint32_t fastHommingVelocity = 5000;
    if (performSDODOWNLOAD(CO->SDOclient,0x6099,1,&fastHommingVelocity,sizeof(fastHommingVelocity)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    uint32_t slowhommingvelocity = 1000;
    if (performSDODOWNLOAD(CO->SDOclient,0x6099,2,&slowhommingvelocity,sizeof(slowhommingvelocity)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    uint32_t hommingAccel = 10000;
    if (performSDODOWNLOAD(CO->SDOclient,0x609A,0,&hommingAccel,sizeof(hommingAccel)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
     
    uint16_t controlWord;
      controlWord = 0x0006; // shutdown
    if (performSDODOWNLOAD(CO->SDOclient,0x6040,0,&controlWord,sizeof(controlWord)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    HAL_Delay(10);
    
    controlWord = 0x0007; // switch on
    if (performSDODOWNLOAD(CO->SDOclient,0x6040,0,&controlWord,sizeof(controlWord)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
     controlWord = 0x001F; // Enable operation and start homming
    if (performSDODOWNLOAD(CO->SDOclient,0x6040,0,&controlWord,sizeof(controlWord)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    //finish waiting for homming or waiting to finish homming
    uint16_t statusWord;
    do {
 controlWord = 0x0007; // switch on
    if (performSDODOWNLOAD(CO->SDOclient,0x6041,0,&statusWord,sizeof(statusWord)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    HAL_Delay(10);
    } while ( !(statusWord & 0x1000)); //12th bit= home attain
      
      //change to profile position mode
      mode = 1; //profile postion
       controlWord = 0x0007; // switch on
     if (performSDODOWNLOAD(CO->SDOclient,0x6060,0,&mode,sizeof(mode)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    int32_t targetPosition = 100000; //why this value? according pervious repoert i should set it 
   // if (performSDODOWNLOAD(CO->SDOclient,0x607A,0,&targetPosition,sizeof(targetPosition)) != CO_SDO_RT_ok_communicationEnd){
     // Error_Handler();
      if (setMotorPosition(CO->SDOclient,targetPosition) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    uint32_t profileVelocity = 50000;
    if (performSDODOWNLOAD(CO->SDOclient,0x6081,0,&profileVelocity,sizeof(profileVelocity)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    uint32_t  profileAccel = 100000;
    if (performSDODOWNLOAD(CO->SDOclient,0x6083,0,&profileAccel,sizeof(targetPosition)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    uint32_t profileDecel = 100000;
    if (performSDODOWNLOAD(CO->SDOclient,0x6084,0,&profileDecel,sizeof(profileDecel)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
     controlWord = 0x001F; // Enable operation and start 
    if (performSDODOWNLOAD(CO->SDOclient,0x6040,0,&controlWord,sizeof(controlWord)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    }
    
    
   // for PDO position regulation
    uint32_t mapping[] = {0x060020};
     
    CO_TPDO_t TPDO;
   extern OD_t OD;
     OD_entry_t* OD_1800 = OD_find(&OD, 0X1800);
      OD_entry_t* OD_1A00 = OD_find(&OD, 0X1A00);
     OD_set_value(OD_1A00,1,mapping,sizeof(mapping), true);
      CO_TPDO_init(&TPDO,&OD, CO ->em, CO ->SYNC,0X180 +canOpenNodeSTM32.desiredNodeID, OD_1800, OD_1A00,CO ->CANmodule, 0, NULL);
    
      
    //regulation the driver
        uint32_t cobId = 0x180 + 1; //FROM node1 OF TPDO1 TO cobid
        if (performSDODOWNLOAD(CO->SDOclient,0x1800,1,&cobId,sizeof(cobId)) != CO_SDO_RT_ok_communicationEnd){
      Error_Handler();
    } 
  
    CO_CANsetNormalMode(CO ->CANmodule);
    
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    
    canopen_app_process();
    if (setMotorPosition(CO->SDOclient,5000) != CO_SDO_RT_ok_communicationEnd){
    Error_Handler();
    }
        HAL_Delay(1000);
       if (setMotorPosition(CO->SDOclient,10000) != CO_SDO_RT_ok_communicationEnd){
    Error_Handler();
    }
        HAL_Delay(1000);
  
    
    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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

  /* USER CODE END CAN_Init 2 */

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
  htim2.Init.Prescaler = 7199;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
