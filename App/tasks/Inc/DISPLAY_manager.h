/**
 * @file        DISPLAY_manager.h
 * @brief       Header file for display manager using FreeRTOS.
 *              Declares the display control task and its attributes.
 *
 * @authors     Mateusz Turycz
 *              Aleksander Uliczny
 *
 * @date        2025-05-27
 * @version     1.0
 */

#pragma once /**< Ensure this header is included only once during compilation */

/** @file display_manager.h
 *  @brief Header file for the DISPLAY manager thread.
 *  @details This file declares the DISPLAY manager thread and its attributes,
 *           used for controlling display output in the STM32 RTOS application.
 */

#include "FreeRTOS.h"  /**< FreeRTOS base includes */
#include "cmsis_os2.h" /**< CMSIS-RTOS2 API includes */
#include "lcd.h"      /**< LCD display control functions */
#include "utils.h"

extern osThreadId_t DISPLAY_manager_handle;

extern const osThreadAttr_t DISPLAY_attributes;
struct DISPLAY_args
{
    MUTEX_f *CURRENT_TEMP;
    MUTEX_f *TARGET_TEMP;
    MUTEX_uint8 *heater_state;
    MUTEX_uint8 *USB_connected;
};
/**
 * @brief Thread function for managing display updates.
 * @param argument Pointer to thread argument (if any).
 */
void DISPLAY_manager(void *argument);


