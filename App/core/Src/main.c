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
// #include "usbd_cdc_if.h"
// #include "C:\Users\M\Desktop\STMprojects\USB\Api\core\src"

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

void MAIN_task(void *argument);
// void Master(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t DataToSend[16];
uint8_t MessageCounter = 0;
uint8_t MessageLength = 0;

MessageBufferHandle_t receive_queue;
MessageBufferHandle_t send_queue;
MUTEX_uint8 USB_connected = {.semaphore = NULL, .value = 0}; // Semaphore for USB connection status
static struct COM_args COM_args = {
    .receive_queue = NULL,      // Will be set in main
    .send_queue = NULL,         // Will be set in main
    .connected = &USB_connected // Pointer to USB_connected variable
};

MUTEX_f CURRENT_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_f TARGET_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_uint8 HEATER_STATE = {.semaphore = NULL, .value = 0};
static struct TEMP_args TEMP_args = {
    .CURRENT_TEMP = &CURRENT_TEMP,
    .TARGET_TEMP = &TARGET_TEMP,
    .heater_state = &HEATER_STATE};

MUTEX_f OBJECT_DISTANCE = {.semaphore = NULL, .value = 0};
static struct TOF_args TOF_args = {
    .distance = &OBJECT_DISTANCE};

static struct DISPLAY_args DISPLAY_args = {
    .CURRENT_TEMP = &CURRENT_TEMP,
    .TARGET_TEMP = &TARGET_TEMP,
    .heater_state = &HEATER_STATE,
    .USB_connected = &USB_connected};

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  USB_connected.value = 0;
  // uint8_t* usb_ptr = &USB_connected; // Pointer to USB_connected variable

  send_queue = xMessageBufferCreate(sizeof(DataToSend) * sizeof(uint8_t) * 16);
  receive_queue = xMessageBufferCreate(sizeof(DataToSend) * sizeof(uint8_t) * 16);

  RTT(0, "%lu\r\n", &USB_connected);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  USB_connected.semaphore = xSemaphoreCreateMutex();
  TARGET_TEMP.semaphore = xSemaphoreCreateMutex();
  CURRENT_TEMP.semaphore = xSemaphoreCreateMutex();
  HEATER_STATE.semaphore = xSemaphoreCreateMutex();
  OBJECT_DISTANCE.semaphore = xSemaphoreCreateMutex();
  BLUE_LED.semaphore = xSemaphoreCreateMutex();
  RED_LED.semaphore = xSemaphoreCreateMutex();
  GREEN_LED.semaphore = xSemaphoreCreateMutex();

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

  HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, 1);
  HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, 1);
  HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, 1);

  COM_args.receive_queue = &receive_queue; // Set the receive queue in COM_args
  COM_args.send_queue = &send_queue;       // Set the send queue in COM_args
  COM_args.connected = &USB_connected;     // Set the USB_connected semaphore in COM_args
  COM_manager_handle = osThreadNew(COM_manager, &COM_args, &COM_attributes);

  TEMP_manager_handle = osThreadNew(TEMP_manager, &TEMP_args, &TEMP_attributes);

  TOF_args.distance = &OBJECT_DISTANCE; // Set the distance mutex in TOF_args
  TOF_manager_handle = osThreadNew(TOF_manager, &TOF_args, &TOF_attributes);

  DISPLAY_manager_handle = osThreadNew(DISPLAY_manager, &DISPLAY_args, &DISPLAY_attributes);

  osThreadNew(MAIN_task, NULL, &(const osThreadAttr_t){
                                   .name = "MAIN",
                                   .stack_size = 128 * 4,
                                   .priority = (osPriority_t)osPriorityNormal,
                               });

  // LED_manager_handle = osThreadNew(LED_manager, NULL, &LED_attributes);

  RTT(0, "starting scheduler \r \n ");

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

void MAIN_task(void *argument)
{
  uint8_t status;
  uint8_t cmd;
  uint8_t len;
  uint8_t payload[32];          // bufor na dane przychodzące
  uint8_t response[64];         // bufor na dane wychodzące
  uint8_t packet[RX_BUFF_SIZE]; // bufor na dane przychodzące z USB
  printf("main task started \r\n");

  uint32_t last_check = HAL_GetTick();

  while (1)
  {
    if (xMessageBufferIsEmpty(receive_queue) == pdFALSE)
    {
      status = xMessageBufferReceive(receive_queue, packet, RX_BUFF_SIZE, portMAX_DELAY);
      int result = parse_packet(packet, RX_BUFF_SIZE, &cmd, payload, &len);

      if (result == 0)
      {
        switch (cmd)
        {
        case SET_LED_STATE:
        {
          if (len < 2)
            break;
          uint8_t led_nr = payload[0];
          uint8_t state = payload[1];

          const char *msg;
          if (led_nr == 0x01)
          {
            while (!xSemaphoreTake(RED_LED.semaphore, portMAX_DELAY))
              ;
            HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, !state);
            xSemaphoreGive(RED_LED.semaphore);
            msg = "RL switched";
          }
          else if (led_nr == 0x02)
          {
            while (!xSemaphoreTake(GREEN_LED.semaphore, portMAX_DELAY))
              ;
            HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, !state);
            xSemaphoreGive(GREEN_LED.semaphore);

            msg = "GL switched";
          }
          else
          {
            msg = "unknown LED";
          }

          create_packet(response, SET_LED_STATE, (uint8_t *)msg, strlen(msg));
          xMessageBufferSend(send_queue, response, 4 + strlen(msg), portMAX_DELAY);
          break;
        }

        case REQUEST_CURRENT_TEMPERATURE:
        {
          while (!xSemaphoreTake(CURRENT_TEMP.semaphore, portMAX_DELAY))
            ;
          double temp = CURRENT_TEMP.value;
          xSemaphoreGive(CURRENT_TEMP.semaphore);

          create_packet(response, REQUEST_CURRENT_TEMPERATURE, (uint8_t *)&temp, sizeof(temp));
          xMessageBufferSend(send_queue, response, 4 + sizeof(temp), portMAX_DELAY);
          break;
        }

        case SET_TARGET_TEMPERATURE:
        {
          if (len < sizeof(float))
            break;
          float target;
          memcpy(&target, payload, sizeof(float));
          while (!xSemaphoreTake(TARGET_TEMP.semaphore, portMAX_DELAY))
            ;
          TARGET_TEMP.value = target;
          xSemaphoreGive(TARGET_TEMP.semaphore);

          create_packet(response, SET_TARGET_TEMPERATURE, (uint8_t *)&target, sizeof(target));
          xMessageBufferSend(send_queue, response, 4 + sizeof(target), portMAX_DELAY);
          break;
        }

        case REQUEST_CURRENT_HEATER_STATE:
        {
          while (!xSemaphoreTake(HEATER_STATE.semaphore, portMAX_DELAY))
            ;
          create_packet(response, REQUEST_CURRENT_HEATER_STATE, &(HEATER_STATE.value), 1);
          xSemaphoreGive(HEATER_STATE.semaphore);

          xMessageBufferSend(send_queue, response, 5, portMAX_DELAY);
          break;
        }

        default:
        {
          const char *err = "Unknown CMD";
          create_packet(response, 0xFF, (uint8_t *)err, strlen(err));
          xMessageBufferSend(send_queue, response, 4 + strlen(err), portMAX_DELAY);
          break;
        }
        }
      }
      else
      {
        RTT(0, "Bad packet (err %d)\r\n", result);
      }
    }
    if (HAL_GetTick() - last_check > SAFETY_CHECK_TIME)
    {

      while (!xSemaphoreTake(OBJECT_DISTANCE.semaphore, portMAX_DELAY))
        ;
      if (OBJECT_DISTANCE.value > SAFETY_DISTANCE)
      {
        create_packet(response, DISTANCE_ERROR, (uint8_t *)&(OBJECT_DISTANCE.value), sizeof(OBJECT_DISTANCE.value));
        xMessageBufferSend(send_queue, response, 4 + sizeof(OBJECT_DISTANCE.value), portMAX_DELAY);
      }
      xSemaphoreGive(OBJECT_DISTANCE.semaphore);
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
