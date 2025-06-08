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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "SEGGER_RTT_printf.h"
// #include "SEGGER_RTT_printf.c"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "VL53L0X.h"
#include "lcd.h"
#include "stdio.h"
#include "configurables.h"
#include "commands.h"
#include "utils.h"
#include "thermal_control.h"
//#include "usbd_cdc_if.h"
//#include "C:\Users\M\Desktop\STMprojects\USB\Api\core\src"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    SemaphoreHandle_t semaphore;
    double value;
} MUTEX_f;

typedef struct {
    SemaphoreHandle_t semaphore;
    GPIO_TypeDef* port;
    uint16_t pin;
} MUTEX_digitPin;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
osThreadId_t COM_manager_handle;
const osThreadAttr_t COM_attributes = {
        .name = "COM",
        .stack_size = 128 * 2,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t LED_manager_handle;
const osThreadAttr_t LED_attributes = {
        .name = "LED",
        .stack_size = 32,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t TEMP_manager_handle;
const osThreadAttr_t TEMP_attributes = {
        .name = "TEMP",
        .stack_size = 128 * 1,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t TOF_manager_handle;
const osThreadAttr_t TOF_attributes = {
        .name = "TOF",
        .stack_size = 128 * 1,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t DISPLAY_manager_handle;
const osThreadAttr_t DISPLAY_attributes = {
        .name = "DISPLAY",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void COM_manager(void *argument);

void LED_manager(void *argument);

//void Master(void *argument);

void TEMP_manager(void *argument);

void TOF_manager(void *argument);
void DISPLAY_manager(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// RTT(0,function
uint8_t DataToSend[16];
uint8_t MessageCounter = 0;
uint8_t MessageLength = 0;


MUTEX_f ACTUAL_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_f TARGET_TEMP = {.semaphore = NULL, .value = 0};
MUTEX_digitPin BLUE_LED = {.semaphore=NULL, .port = GPIOA, .pin = GPIO_PIN_5};
MUTEX_digitPin RED_LED = {.semaphore=NULL, .port = GPIOA, .pin = GPIO_PIN_6};
MUTEX_digitPin GREEN_LED = {.semaphore=NULL, .port = GPIOA, .pin = GPIO_PIN_7};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
    // SEGGER_SYSVIEW_Conf();
    // SEGGER_SYSVIEW_Start();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    MX_USB_DEVICE_Init();
    // RTT(0,"waiting for connection \r \n ");
    HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, GPIO_PIN_SET);

    uint32_t before = HAL_GetTick();
    while((HAL_GetTick()-before)<CONNECTION_TIMEOUT){

        HAL_GPIO_TogglePin(BLUE_LED.port, BLUE_LED.pin);
        HAL_Delay(100);
        RTT(0,"waiting for connection \r \n ");
        USBD_StatusTypeDef x = USBD_LL_DevConnected(&hUsbDeviceFS);
        if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED){
            enable_potentiometer = 0;
            HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, GPIO_PIN_RESET);
            RTT(0,"connected");
            break;
        }
    };

    if(enable_potentiometer==1){
        USBD_DeInit(&hUsbDeviceFS);
        HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, GPIO_PIN_SET);
    }



    osKernelInitialize();

    if (enable_potentiometer == 1) {
        //    osThreadNew(POT_manager, NULL, &POT_attributes);

    }else
    {
        osThreadNew(COM_manager, NULL, &COM_attributes);

    }

    // COM_manager_handle = osThreadNew(LED_manager, NULL, &LED_attributes);
    // TEMP_manager_handle = osThreadNew(TEMP_manager, NULL, &TEMP_attributes);
  DISPLAY_manager_handle = osThreadNew(DISPLAY_manager, NULL, &DISPLAY_attributes);
  TOF_manager_handle = osThreadNew(TOF_manager, NULL, &TOF_attributes);
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





void COM_manager(void *argument) {
    uint32_t len = 32;
    char feedback[RX_BUFF_SIZE];
    RTT(0,"COM manager started \r \n");
    RTT(0,"before while loop \r \n");
//    extern uint8_t received_bool;
    while (1) {

//RTT(0,"waiting for data \r \n");
        if (hUsbDeviceFS.dev_connection_status == 0x01) {
            HAL_Delay(1);


            RTT(0,"received command: %02X \r \n", UserRxBufferFS[0]);
            for (int i = 0; i < RX_BUFF_SIZE; i++) {
                RTT(0,"%02X ", UserRxBufferFS[i]);
            }
            switch (UserRxBufferFS[0]) {
                case SET_LED_STATE:
                    switch(UserRxBufferFS[RX_BUFF_SIZE - 2]){
                        case 0x01:
                            HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, !UserRxBufferFS[RX_BUFF_SIZE - 1]);
                            strcpy(feedback+(RX_BUFF_SIZE-12), "BL switched");

                            break;
                        case 0x02:
                            HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, !UserRxBufferFS[RX_BUFF_SIZE - 1]);
                            strcpy(feedback+(RX_BUFF_SIZE-12), "EL switched");
                            break;
                        default:
                            RTT(0,"unknown command \r \n");
                            break;
                    }

                    feedback[0]= SET_LED_STATE;
                    CDC_Transmit_FS(feedback, RX_BUFF_SIZE);
                    break;
                case REQUEST_ACTUAL_TEMPERATURE:
                    RTT(0,"sending actual temperature \r \n");
                    char value[RX_BUFF_SIZE];
// Write
                    memset(value, 0x00, RX_BUFF_SIZE);
                    * ((double *) (value + (sizeof(value) - sizeof(double)))) = ACTUAL_TEMP.value;
                    value[0]= REQUEST_ACTUAL_TEMPERATURE;
// Read
//                    int outValue = * ((int *) (set + (sizeof(set) - sizeof(int))));
                    CDC_Transmit_FS(value, RX_BUFF_SIZE);
//                    xSemaphoreGive(ACTUAL_TEMP.semaphore);
                    break;
                case SET_TARGET_TEMPERATURE:
                    RTT(0,"setting target temperature \r \n");
//                    float ee =
                    TARGET_TEMP.value = * ((float *) (UserRxBufferFS + (sizeof(UserRxBufferFS) - sizeof(float))));
                    memset(feedback, 0x00, RX_BUFF_SIZE);
                    * ((double *) (feedback + (sizeof(feedback) - sizeof(double)))) = TARGET_TEMP.value;
                    feedback[0]= SET_TARGET_TEMPERATURE;
                    CDC_Transmit_FS(feedback, RX_BUFF_SIZE);

                    break;
                case REQUEST_ACTUAL_HEATER_STATE:
                    RTT(0,"sending actual heater state \r \n");
                    char h_value[RX_BUFF_SIZE];
// Write
                    memset(h_value, 0x20, RX_BUFF_SIZE);
                    * ((int *) (h_value + (sizeof(h_value) - sizeof(int)))) = ((TIM2->CCR2)/MAX_PWM_VALUE)*100;

// Read
//                    int outValue = * ((int *) (set + (sizeof(set) - sizeof(int))));
                    CDC_Transmit_FS(value, RX_BUFF_SIZE);
                default:
                    RTT(0,"unknown command \r \n");
                    break;
            }

            memset(feedback, 0x20, RX_BUFF_SIZE);
            hUsbDeviceFS.dev_connection_status = 0x00;
//        osDelay(100);
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
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100-0);

    HAL_ADC_Start(&hadc1);
    uint16_t dac_out;
    float prev_error = 0;
    float integral = 0;
    float prev_integral = 0;
    float error = 0;
    uint32_t time_variable=HAL_GetTick();
//    uint32_t time_diff;
    while (1) {


        if(HAL_GetTick()%TEMP_CORRECTION_INTERVAL==0) {
//            xSemaphoreTake(ACTUAL_TEMP.semaphore, portMAX_DELAY);
            HAL_ADC_Start(&hadc1);
            HAL_Delay(10);
            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
                ACTUAL_TEMP.value = calc_temp(HAL_ADC_GetValue(&hadc1));

            }
            HAL_ADC_Stop(&hadc1);

            time_variable = HAL_GetTick()-time_variable;
            error = TARGET_TEMP.value==0 ? 0.0 : (TARGET_TEMP.value - ACTUAL_TEMP.value);
            integral = prev_integral + error * TEMP_CORRECTION_INTERVAL;

//            dac_out = PID_PROPORTIONAL*error + PID_INTEGRAL*integral + PID_DERIVATIVE*((error - prev_error ) / TEMP_CORRECTION_INTERVAL);
            dac_out =  (TARGET_TEMP.value   - ACTUAL_TEMP.value)*100/ACTUAL_TEMP.value;
            prev_error = error;
            prev_integral = integral;
            time_variable = HAL_GetTick();

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dac_out > 100 ? 0:(100-dac_out));




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
    uint8_t prev_bool = hUsbDeviceFS.dev_connection_status;
    RTT(0,"LED manager started \r \n");
    RTT(0,"prev_bool: %d \r \n", prev_bool);
    while (1) {
//if(prev_bool!=hUsbDeviceFS.dev_connection_status){
//            prev_bool = hUsbDeviceFS.dev_connection_status;
//            RTT(0,"bool_state has changed \r \n");
//        }
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


void DISPLAY_manager(void *argument) {
    RTT(0,"DISPLAY manager started \r \n");
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      initLCD();
  char *stl = "Github: @fatay";
  writeLCD(stl);
  char *ttl = "LCD Library";
  setCursor(0,1);

  writeLCD(ttl);
  
    while (1) {
        osDelay(400);
    }
}




void TOF_manager(void *argument) {
    RTT(0,"TOF manager started \r \n");
  char msgBuffer[52];
	for (uint8_t i = 0; i < 52; i++) {
		msgBuffer[i] = ' ';
	}

	// Initialise the VL53L0X
	statInfo_t_VL53L0X distanceStr;
	initVL53L0X(1, &hi2c1);
	uint16_t distance;

	// Configure the sensor for high accuracy and speed in 20 cm.
	setSignalRateLimit(400);
	setVcselPulsePeriod(VcselPeriodPreRange, 10);
	setVcselPulsePeriod(VcselPeriodFinalRange, 14);
	setMeasurementTimingBudget(300 * 1000UL);
	while (1) {

		distance = readRangeSingleMillimeters(&distanceStr);

    RTT(0, "Distance: %d mm\r\n", distance);

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
