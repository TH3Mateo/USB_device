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

#pragma once  /**< Ensure this header is included only once during compilation */

/** @file display_manager.h
 *  @brief Header file for the DISPLAY manager thread.
 *  @details This file declares the DISPLAY manager thread and its attributes,
 *           used for controlling display output in the STM32 RTOS application.
 */

#include "FreeRTOS.h"   /**< FreeRTOS base includes */
#include "cmsis_os2.h"  /**< CMSIS-RTOS2 API includes */

/**
 * @brief Thread function for managing display updates.
 * @param argument Pointer to thread argument (if any).
 */
void DISPLAY_manager(void *argument);
>>>>>>> db184d3 (Code comments)

/**
 * @brief Handle for the DISPLAY manager thread.
 */
extern osThreadId_t DISPLAY_manager_handle;
