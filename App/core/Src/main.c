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
#include "message_buffer.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "SEGGER_RTT_printf.h"
// #include "SEGGER_RTT_printf.c"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lcd.h"
#include "stdio.h"
#include "configurables.h"
#include "commands.h"
#include "utils.h"
#include "thermal_control.h"

#include "TOF_manager.h"
#include "TEMP_manager.h"
#include "DISPLAY_manager.h"
#include "COM_manager.h"
//#include "usbd_cdc_if.h"
//#include "C:\Users\M\Desktop\STMprojects\USB\Api\core\src"

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

/* USER CODE BEGIN PV */







/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */


//void Master(void *argument);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// RTT(0,function
uint8_t DataToSend[16];
uint8_t MessageCounter = 0;
uint8_t MessageLength = 0;

MessageBufferHandle_t send_queue;
MessageBufferHandle_t receive_queue;
uint8_t USB_connected = 0;

MUTEX_f CURRENT_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_f TARGET_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_uint8 HEATER_STATE = {.semaphore = NULL, .value = 0}; 

MUTEX_f OBJECT_DISTANCE = {.semaphore = NULL, .value = 0};



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
MessageBufferHandle_t send_queue = xMessageBufferCreate(sizeof(DataToSend) * 8);
MessageBufferHandle_t receive_queue = xMessageBufferCreate(sizeof(DataToSend) * 8);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    // SEGGER_SYSVIEW_Conf();
    // SEGGER_SYSVIEW_Start();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    // RTT(0,"waiting for connection \r \n ");
    HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, GPIO_PIN_SET);

    // uint32_t before = HAL_GetTick();
    // while((HAL_GetTick()-before)<CONNECTION_TIMEOUT){

    //     HAL_GPIO_TogglePin(BLUE_LED.port, BLUE_LED.pin);
    //     HAL_Delay(100);
    //     RTT(0,"waiting for connection \r \n ");
    //     USBD_StatusTypeDef x = USBD_LL_DevConnected(&hUsbDeviceFS);
    //     if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED){
    //         enable_potentiometer = 0;
    //         HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, GPIO_PIN_RESET);
    //         RTT(0,"connected");
    //         break;
    //     }
    // };

    // if(enable_potentiometer==1){
    //     USBD_DeInit(&hUsbDeviceFS);
    //     HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, GPIO_PIN_SET);
    // }

    // osKernelInitialize();

    // if (enable_potentiometer == 1) {
    //     //    osThreadNew(POT_manager, NULL, &POT_attributes);

    // }else
    // {
    //     COM_manager_handle= osThreadNew(COM_manager, NULL, &COM_attributes);

    // }
// struct COM_args *COM_args = pvPortMalloc(sizeof(struct COM_args));  // FreeRTOS heap

// COM_args->receive_queue = receive_queue;
// COM_args->send_queue = send_queue;
// COM_args->connected = &USB_connected;
//     COM_manager_handle = osThreadNew(COM_manager, &COM_args, &COM_attributes);


// struct TEMP_args *TEMP_args = pvPortMalloc(sizeof(struct TEMP_args));  // FreeRTOS heap
// TEMP_args->CURRENT_TEMP = &CURRENT_TEMP;
// TEMP_args->TARGET_TEMP = &TARGET_TEMP;
// TEMP_args->heater_state = &HEATER_STATE;
//     TEMP_manager_handle = osThreadNew(TEMP_manager, &TEMP_args, &TEMP_attributes);
  // DISPLAY_manager_handle = osThreadNew(DISPLAY_manager, NULL, &DISPLAY_attributes);
    struct TOF_args TOF_args = {
        .distance = &OBJECT_DISTANCE
    };
    TOF_manager_handle = osThreadNew(TOF_manager, &TOF_args, &TOF_attributes);
    // LED_manager_handle = osThreadNew(LED_manager, NULL, &LED_attributes);

    // RTT(0,"starting scheduler \r \n ");
    // SEGGER_SYSVIEW_Print("starting scheduler \r \n ");
    // SEGGER_SYSVIEW_Start();

    
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





void MAIN_task(void *argument) {
    uint8_t status;
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[32];     // bufor na dane przychodzące
    uint8_t response[64];    // bufor na dane wychodzące

    printf("COM manager started \r\n");

    while (1) {
        if (hUsbDeviceFS.dev_connection_status == 0x01) {
            HAL_Delay(1);

            int result = parse_packet(UserRxBufferFS, RX_BUFF_SIZE, &cmd, payload, &len);

            if (result == 0) {
                switch (cmd) {
                    case SET_LED_STATE: {
                        if (len < 2) break;
                        uint8_t led_nr = payload[0];
                        uint8_t state = payload[1];

                        const char *msg;
                        if (led_nr == 0x01) {
                            HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, !state);
                            msg = "BL switched";
                        } else if (led_nr == 0x02) {
                            HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, !state);
                            msg = "EL switched";
                        } else {
                            msg = "unknown LED";
                        }

                        create_packet(response, SET_LED_STATE, (uint8_t *)msg, strlen(msg));
                        xMessageBufferSend(send_queue, response, 4 + strlen(msg), portMAX_DELAY);
                        break;
                    }

                    case REQUEST_CURRENT_TEMPERATURE: {
                        double temp = CURRENT_TEMP.value;
                        create_packet(response, REQUEST_CURRENT_TEMPERATURE, (uint8_t *)&temp, sizeof(temp));
                        xMessageBufferSend(send_queue, response, 4 + sizeof(temp), portMAX_DELAY);
                        break;
                    }

                    case SET_TARGET_TEMPERATURE: {
                        if (len < sizeof(float)) break;
                        float target;
                        memcpy(&target, payload, sizeof(float));
                        TARGET_TEMP.value = target;

                        create_packet(response, SET_TARGET_TEMPERATURE, (uint8_t *)&target, sizeof(target));
                        xMessageBufferSend(send_queue, response, 4 + sizeof(target), portMAX_DELAY);
                        break;
                    }

                    case REQUEST_CURRENT_HEATER_STATE: {
                        uint8_t heater_state = 25; // przykładowa wartość
                        create_packet(response, REQUEST_CURRENT_HEATER_STATE, &heater_state, 1);
                        xMessageBufferSend(send_queue, response, 5, portMAX_DELAY);
                        break;
                    }

                    default: {
                        const char *err = "Unknown CMD";
                        create_packet(response, 0xFF, (uint8_t *)err, strlen(err));
                        xMessageBufferSend(send_queue, response, 4 + strlen(err), portMAX_DELAY);
                        break;
                    }
                }
            } else {
                RTT(0,"Bad packet (err %d)\r\n", result);
            }
        }
    }
}









/* USER CODE END 4 */

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
  if (htim->Instance == TIM11)
  {
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
     ex: RTT(0,"Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
