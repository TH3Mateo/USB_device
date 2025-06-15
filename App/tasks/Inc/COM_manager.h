/**
 * @file        COM_manager.h
 * @brief       Header file for USB communication manager using FreeRTOS.
 *              Defines the communication task and associated data structures.
 * 
 * @authors     Mateusz Turycz  
 *              Aleksander Uliczny
 * 
 * @date        2025-05-21
 * @version     1.0
 */

#pragma once

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "utils.h"
#include "message_buffer.h"

#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usb_device.h"

/**
 * @brief USB device handle (defined in USB device middleware).
 */
extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief Handle for the COM manager thread.
 */
extern osThreadId_t COM_manager_handle;

/**
 * @brief Thread attributes for the COM manager.
 */
extern const osThreadAttr_t COM_attributes;

/**
 * @brief Structure holding arguments for the COM manager thread.
 */
struct COM_args {
    MessageBufferHandle_t* receive_queue; /**< Pointer to the message buffer for received data */
    MessageBufferHandle_t* send_queue;    /**< Pointer to the message buffer for data to send */
    MUTEX_uint8 *connected;               /**< Pointer to connection status flag */
};

/**
 * @brief Thread function for managing USB communication (CDC).
 * 
 * @param args Pointer to a COM_args structure containing queues and status flag.
 */
void COM_manager(void *args);
