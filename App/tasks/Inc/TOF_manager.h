/**
 * @file        TOF_manager.h
 * @brief       Header file for Time-of-Flight (TOF) sensor manager using FreeRTOS.
 *              Declares the TOF measurement task and related data structures.
 * 
 * @authors     Mateusz Turycz  
 *              Aleksander Uliczny
 * 
 * @date        2025-05-24
 * @version     1.0
 */

#pragma once

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "utils.h"

/**
 * @brief Handle for the TOF manager thread.
 */
extern osThreadId_t TOF_manager_handle;

/**
 * @brief Thread attributes for the TOF manager.
 */
extern const osThreadAttr_t TOF_attributes;

/**
 * @brief Structure containing arguments for the TOF manager task.
 */
struct TOF_args {
    MUTEX_f *distance; /**< Mutex-protected pointer to measured distance value */
    // Add other parameters if needed
};

/**
 * @brief Thread function for managing Time-of-Flight distance measurements.
 * 
 * @param argument Pointer to TOF_args structure with sensor data references.
 */
void TOF_manager(void *argument);
