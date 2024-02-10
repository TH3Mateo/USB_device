/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "stdio.h"
#include "configurables.h"
#include "commands.h"
#include "utils.h"
#include "thermal_control.h"
//#include "C:\Users\M\Desktop\STMprojects\USB\Api\core\src"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char *ptr, int len) {
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        ITM_SendChar((*ptr++));
    }
    return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
I2C_HandleTypeDef hi2c1;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    SemaphoreHandle_t semaphore;
    float value;
} MUTEX_f;

typedef struct {
    SemaphoreHandle_t semaphore;
    GPIO_TypeDef* port;
    uint16_t pin;
} MUTEX_digitPin;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

osThreadId_t COM_manager_attributes;
const osThreadAttr_t COM_attributes = {
        .name = "COM",
        .stack_size = 128 * 2,
        .priority = (osPriority_t) osPriorityAboveNormal2,
};

osThreadId_t LED_manager_attributes;
const osThreadAttr_t LED_attributes = {
        .name = "LED",
        .stack_size = 32,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t TEMP_manager_attributes;
const osThreadAttr_t TEMP_attributes = {
        .name = "TEMP",
        .stack_size = 128 * 1,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t POT_manager_attributes;
const osThreadAttr_t POT_attributes = {
        .name = "POT",
        .stack_size = 128 * 1,
        .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void COM_manager(void *argument);

void LED_manager(void *argument);

//void Master(void *argument);

void TEMP_manager(void *argument);

void POT_manager(void *argument);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */



uint8_t DataToSend[16];
uint8_t MessageCounter = 0;
uint8_t MessageLength = 0;


MUTEX_f ACTUAL_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_f TARGET_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_digitPin BUILTIN_LED = {.semaphore=NULL, .port = GPIOC, .pin = GPIO_PIN_13};
MUTEX_digitPin EXTERN_LED = {.semaphore=NULL, .port = GPIOA, .pin = GPIO_PIN_0};
uint8_t CDC_RX_Buffer[RX_BUFF_SIZE];
uint8_t CDC_TX_Buffer[RX_BUFF_SIZE];

int main(void) {

    /* USER CODE BEGIN 1 */
    uint8_t enable_potentiometer = 1;



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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
    MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

    uint32_t before = HAL_GetTick();
    while((HAL_GetTick()-before)<CONNECTION_TIMEOUT){

        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
        HAL_Delay(100);
        printf("waiting for connection \r \n ");
        USBD_StatusTypeDef x = USBD_LL_DevConnected(&hUsbDeviceFS);
        if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED){
            enable_potentiometer = 0;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            break;
        }
    };
    if(enable_potentiometer==1){
        USBD_DeInit(&hUsbDeviceFS);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

    osKernelInitialize();

    if (enable_potentiometer == 1) {
        //    osThreadNew(POT_manager, NULL, &POT_attributes);

    }else
    {
        osThreadNew(COM_manager, NULL, &COM_attributes);

    }

    osThreadNew(LED_manager, NULL, &LED_attributes);
//    osThreadNew(TEMP_manager, NULL, &TEMP_attributes);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA6 PA7 PA8 PA9
                           PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB10 PB11
                           PB12 PB14 PB15 PB4
                           PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//void Master(void *argument) {
//    osThreadId blinking = osThreadNew(LED_manager, NULL, LED_manager_attributes);
//    osThreadId send = osThreadNew(COM_manager, NULL, COM_manager_attributes);
//    osThreadResume(send);
//    printf("\r \n \r");
//
//    while (1) {
//
//        osDelay(3000);
//        osThreadSuspend(blinking);
//        printf("turned off\n \r");
//        osDelay(3000);
//        osThreadResume(blinking);
//        printf("turned on \r \n");
//
//    }
//}




void COM_manager(void *argument) {
    uint32_t len = 32;
    uint8_t feedback[RX_BUFF_SIZE];
    printf("COM manager started \r \n");
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, CDC_RX_Buffer);
    printf("before while loop \r \n");
//    extern uint8_t received_bool;
    while (1) {

//printf("waiting for data \r \n");
        if(hUsbDeviceFS.received_flag==0x01){


            printf("received command: %02X \r \n", CDC_RX_Buffer[0]);
            for (int i = 0; i < RX_BUFF_SIZE; i++) {
                printf("%02X ", CDC_RX_Buffer[i]);
            }
            switch(CDC_RX_Buffer[0]) {
                case SET_LED_STATE:
                    printf("switching LED1 \r \n");
                    HAL_GPIO_WritePin(BUILTIN_LED.port, BUILTIN_LED.pin, CDC_RX_Buffer[RX_BUFF_SIZE-2]);
                    strcpy(feedback, "BL switched ");
                    feedback[RX_BUFF_SIZE-2] = CDC_RX_Buffer[RX_BUFF_SIZE-2];
                    CDC_Transmit_FS(feedback, RX_BUFF_SIZE);
                    break;
                case SET_LED2_STATE:
//                    xSemaphoreTake(EXTERN_LED.semaphore, portMAX_DELAY);
                    printf("switching LED2 \r \n");

                    HAL_GPIO_WritePin(EXTERN_LED.port, EXTERN_LED.pin, CDC_RX_Buffer[RX_BUFF_SIZE-2]);
                    strcpy(feedback, "EL switched ");

                    feedback[RX_BUFF_SIZE-2] = CDC_RX_Buffer[RX_BUFF_SIZE-2];
                    CDC_Transmit_FS(feedback, RX_BUFF_SIZE);
//                    xSemaphoreGive(EXTERN_LED.semaphore);
                    break;
                case REQUEST_ACTUAL_TEMPERATURE:
                    xSemaphoreTake(ACTUAL_TEMP.semaphore, portMAX_DELAY);
                    printf("sending actual temperature \r \n");
                    CDC_Transmit_FS((uint8_t *) &ACTUAL_TEMP.value, sizeof(ACTUAL_TEMP.value));
                    xSemaphoreGive(ACTUAL_TEMP.semaphore);
                case SET_TARGET_TEMPERATURE:
                    printf("setting target temperature \r \n");
                    xSemaphoreTake(TARGET_TEMP.semaphore, 60);
                    TARGET_TEMP.value = HexToDec(CDC_RX_Buffer+1, 4);
                    strcpy(feedback, "target temp set ");
                    CDC_Transmit_FS(feedback, RX_BUFF_SIZE);
                    xSemaphoreGive(TARGET_TEMP.semaphore);

                    break;
//                case START_SENDING_PROGRAM:
//                    break;


            }

        }

    }
}


void POT_manager(void *argument) {
    uint16_t potentiometer_value = HAL_ADC_GetValue(&hadc1);
    while(1) {
    HAL_Delay(1000);

    };
}

void TEMP_manager(void *argument) {
    uint16_t dac_out;
    float prev_error = 0;
    float integral = 0;
    float prev_integral = 0;
    float error = 0;
    uint32_t time_variable=HAL_GetTick();
    while (1) {


        if(HAL_GetTick()-time_variable>=TEMP_CORRECTION_INTERVAL) {
            xSemaphoreTake(ACTUAL_TEMP.semaphore, portMAX_DELAY);
            float measured_temp = calc_temp(HAL_ADC_GetValue(&hadc1));

            time_variable = HAL_GetTick()-time_variable;
            error =(measured_temp- ACTUAL_TEMP.value);
            error = TARGET_TEMP.value - measured_temp;
            integral = prev_integral + error * TEMP_CORRECTION_INTERVAL;

            dac_out = PID_PROPORTIONAL*error + PID_INTEGRAL*integral + PID_DERIVATIVE*((error - prev_error ) / TEMP_CORRECTION_INTERVAL);
            prev_error = error;
            prev_integral = integral;
            time_variable = HAL_GetTick();


            TIM1->CCR1 = dac_out*MAX_PWM_VALUE;


        }
        }
    }

#if LED1_BINARY_BLINK==0x01
void LED_manager(void *argument) {
    while (1) {
        if(received_bool==1){
            unsigned char order[8];
            HexToBin(CDC_RX_Buffer[0], order);
            xSemaphoreTake(LED1.semaphore, portMAX_DELAY);
            for (int i = 7; i >= 0; i--) {
                HAL_Delay(10);
                HAL_GPIO_WritePin(LED1.port, LED1.pin, order[i]);
            }
            xSemaphoreGive(LED1.semaphore);
            while (received_bool==1){
                HAL_Delay(10);
            }

        }



    }
}
#else
//void LED_manager(void *argument) {
//    while (1) {
//        if(received_bool==1){
//            xSemaphoreTake(LED1.semaphore, portMAX_DELAY);
//            HAL_GPIO_WritePin(LED1.port, LED1.pin, CDC_RX_Buffer[0]);
//            xSemaphoreGive(LED1.semaphore);
//            while (received_bool==1){
//                HAL_Delay(10);
//            }
//
//        }
//

void LED_manager(void *argument) {
    uint8_t prev_bool = hUsbDeviceFS.received_flag;
    printf("LED manager started \r \n");
    printf("prev_bool: %d \r \n", prev_bool);
    while (1) {
if(prev_bool!=hUsbDeviceFS.received_flag){
            prev_bool = hUsbDeviceFS.received_flag;
            printf("bool_state has changed \r \n");
        }
////            xSemaphoreTake(LED2.semaphore, portMAX_DELAY);
//            HAL_GPIO_TogglePin(LED2.port, LED2.pin);
////            xSemaphoreGive(LED2.semaphore);
//            HAL_Delay(200);


    }
}
#endif

#if LED1_HEATER_INFO==0x01
void LED_manager(void *argument) {
    while (1) {

    }
}

#endif

#if LED2_CONNECTION_INFO==0x01


#endif






/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
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
