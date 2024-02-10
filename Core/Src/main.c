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
#include "usbd_cdc_if.h"

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

/* Definitions for Blink */

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartBlink(void *argument);

/* USER CODE BEGIN PFP */
void COM_manager(void *argument);

void LED_manager(void *argument);

//void Master(void *argument);

void TEMP_manager(void *argument);

void POT_manager(void *argument);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// printf function

/* USER CODE END 0 */



uint8_t DataToSend[16];
uint8_t MessageCounter = 0;
uint8_t MessageLength = 0;


MUTEX_f ACTUAL_TEMP = {.semaphore = NULL, .value = 999};
MUTEX_f TARGET_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_digitPin BUILTIN_LED = {.semaphore=NULL, .port = GPIOC, .pin = GPIO_PIN_13};
MUTEX_digitPin EXTERN_LED = {.semaphore=NULL, .port = GPIOA, .pin = GPIO_PIN_1};
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
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            printf("waiting for connection \r \n");
            break;
        }
    };
    if(enable_potentiometer==1){
        USBD_DeInit(&hUsbDeviceFS);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    }



    osKernelInitialize();

    if (enable_potentiometer == 1) {
        //    osThreadNew(POT_manager, NULL, &POT_attributes);

    }else
    {
        osThreadNew(COM_manager, NULL, &COM_attributes);

    }

    osThreadNew(LED_manager, NULL, &LED_attributes);
    osThreadNew(TEMP_manager, NULL, &TEMP_attributes);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
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
/**
* @}
*/
/**
* @}
*/

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



}



/* USER CODE BEGIN 4 */





void COM_manager(void *argument) {
    uint32_t len = 32;
    uint8_t feedback[RX_BUFF_SIZE];
    printf("COM manager started \r \n");
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, CDC_RX_Buffer);
    printf("before while loop \r \n");
//    extern uint8_t received_bool;
    while (1) {

//printf("waiting for data \r \n");
        if (hUsbDeviceFS.received_flag == 0x01) {


            printf("received command: %02X \r \n", CDC_RX_Buffer[0]);
            for (int i = 0; i < RX_BUFF_SIZE; i++) {
                printf("%02X ", CDC_RX_Buffer[i]);
            }
            switch (CDC_RX_Buffer[0]) {
                case SET_LED_STATE:
                    printf("switching LED BUILTIN \r \n");
                    HAL_GPIO_WritePin(EXTERN_LED.port, EXTERN_LED.pin, CDC_RX_Buffer[RX_BUFF_SIZE - 2]);
                    strcpy(feedback, "BL switched ");
                    feedback[RX_BUFF_SIZE - 4]= CDC_RX_Buffer[RX_BUFF_SIZE - 3];
                    CDC_Transmit_FS(feedback, RX_BUFF_SIZE);
                    break;
                case REQUEST_ACTUAL_TEMPERATURE:
                    printf("sending actual temperature \r \n");
                    char value[RX_BUFF_SIZE];
                    memset(value, 0x20, RX_BUFF_SIZE);
                    strcpy(value,(uint8_t *) &ACTUAL_TEMP.value);
                    CDC_Transmit_FS(value, RX_BUFF_SIZE);
//                    xSemaphoreGive(ACTUAL_TEMP.semaphore);
                    break;
                case SET_TARGET_TEMPERATURE:
                    printf("setting target temperature \r \n");
                    xSemaphoreTake(TARGET_TEMP.semaphore, 60);
                    TARGET_TEMP.value = HexToDec(CDC_RX_Buffer + 1, 4);
                    strcpy(feedback, "target temp set ");
                    CDC_Transmit_FS(feedback, RX_BUFF_SIZE);
                    xSemaphoreGive(TARGET_TEMP.semaphore);

                    break;
                default:
                    printf("unknown command \r \n");
                    break;
            }
//            CDC_RX_Buffer[0]=0;
            memset(feedback, 0x20, RX_BUFF_SIZE);
            hUsbDeviceFS.received_flag = 0x00;
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
            ACTUAL_TEMP.value = calc_temp(HAL_ADC_GetValue(&hadc1));

            time_variable = HAL_GetTick()-time_variable;
            error = TARGET_TEMP.value - ACTUAL_TEMP.value;
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

/* USER CODE BEGIN Header_StartBlink */
/**
  * @brief  Function implementing the Blink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlink */

/* USER CODE END 5 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
    while (1) {
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
