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
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "stdio.h"
#include "configurables.h"
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
USBD_HandleTypeDef USB1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* Definitions for Blink */

/* USER CODE BEGIN PV */
const osThreadAttr_t Master_attributes = {
        .name = "Master",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityAboveNormal3,
};

osThreadId_t COM_manager_attributes;
const osThreadAttr_t COM_attributes = {
        .name = "COM",
        .stack_size = 128 * 2,
        .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t LED_manager_attributes;
const osThreadAttr_t LED_attributes = {
        .name = "LED",
        .stack_size = 128 * 1,
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




/* USER CODE BEGIN PFP */
void COM_manager(void *argument);

void LED_manager(void *argument);

void Master(void *argument);

void TEMP_manager(void *argument);

void POT_manager(void *argument);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// printf function

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint8_t DataToSend[40];
uint8_t MessageCounter = 0;
uint8_t MessageLength = 0;


int main(void) {
    uint8_t enable_potentiometer = 0;

    /* USER CODE BEGIN 1 */
    uint32_t before = HAL_GetTick();
    while(HAL_GetTick()-before<CONNECTION_TIMEOUT){
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
        HAL_Delay(100);
        if(USBD_LL_DevConnected(&USB1)==HAL_OK){
            enable_potentiometer = 1;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            break;
        }
    };

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();


    /* Configure the system clock */
    SystemClock_Config();


    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USB_DEVICE_Init();


    /* USER CODE BEGIN 2 */


    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    for (int i = 0; i < 5; i++) {
        HAL_Delay(200);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    }

    osKernelInitialize();


    osThreadNew(COM_manager, NULL, &COM_attributes);
    osThreadNew(LED_manager, NULL, &LED_attributes);
//    osThreadNew(TEMP_manager, NULL, &TEMP_attributes);
//    osThreadNew(POT_manager, NULL, &POT_attributes);
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
void SystemClock_Config(void) {
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

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
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA1 PA2 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void Master(void *argument) {
    osThreadId blinking = osThreadNew(LED_manager, NULL, LED_manager_attributes);
    osThreadId send = osThreadNew(COM_manager, NULL, COM_manager_attributes);
    osThreadResume(send);
    printf("\r \n \r");

    while (1) {

        osDelay(3000);
        osThreadSuspend(blinking);
        printf("turned off\n \r");
        osDelay(3000);
        osThreadResume(blinking);
        printf("turned on \r \n");

    }
}


void COM_manager(void *argument) {
    while (1) {
        MessageLength = sprintf(DataToSend, "Wiadomosc nr %d\n\r", MessageCounter);
        CDC_Transmit_FS(DataToSend, MessageLength);
        osDelay(1000);
    }
}

#if LED1_BINARY_BLINK==0x01
void LED_manager(void *argument) {
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
        osDelay(100);
    }
}
#endif

#if LED1_HEATER_INFO==0x01


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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void Error_Handler(void) {
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
