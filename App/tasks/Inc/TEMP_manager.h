/**
 * @file        TEMP_manager.h
 * @brief       Header file for temperature control manager using FreeRTOS.
 *              Declares the temperature regulation task and related data structures.
 * 
 * @authors     Mateusz Turycz  
 *              Aleksander Uliczny
 * 
 * @date        2025-06-02
 * @version     1.0
 */

#pragma once

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "utils.h"

/**
 * @brief Handle for the TEMP manager thread.
 */
extern osThreadId_t TEMP_manager_handle;

/**
 * @brief Thread attributes for the TEMP manager.
 */
extern const osThreadAttr_t TEMP_attributes;

/**
 * @brief Structure containing arguments for the temperature manager task.
 */
struct TEMP_args {
    MUTEX_f *CURRENT_TEMP;   /**< Mutex-protected pointer to current temperature value */
    MUTEX_f *TARGET_TEMP;    /**< Mutex-protected pointer to target temperature value */
    MUTEX_uint8 *heater_state; /**< Mutex-protected pointer to heater state (on/off) */
};

/**
 * @brief Thread function for managing temperature control.
 * 
 * @param arguments Pointer to TEMP_args structure with temperature and heater references.
 */
void TEMP_manager(void* arguments);
