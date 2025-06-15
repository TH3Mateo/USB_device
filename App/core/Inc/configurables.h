/**
 * @file configurables.h
 * @brief Configuration constants and macros for the thermal control system.
 * 
 * @details
 * This header file defines all the configurable constants and macros used
 * throughout the application, including PID parameters, communication settings,
 * timing intervals, and device-specific addresses.
 * 
 * @authors Mateusz Turycz and Aleksander Uliczny
 * @date 2025-05-21
 */

//
// Created by Mateusz Turycz and Aleksander Uliczny on 2025-05-21.
//

#pragma once

/** 
 * @brief Time between retries to connect to USB host in ms.
 */
#define RECONNECTION_TIMEOUT 2500 

/** 
 * @brief Time delay between activity on top of the device in ms.
 */
#define SAFETY_CHECK_TIME 100

/** 
 * @brief Distance in mm that is considered an activity on the device.
 */
#define SAFETY_DISTANCE 150

#define DEVICE_ID 0xFF
#define CONNECTION_TIMEOUT 4000
#define RX_BUFF_SIZE 16

/** 
 * @brief Start byte for each packet.
 */
#define START_BYTE 0xAA

/** 
 * @brief Max packet size for USB communication.
 */
#define MAX_PACKET_SIZE 64

/** 
 * @brief PID parameters for temperature control.
 */
#define PID_PROPORTIONAL 3.0f
#define PID_INTEGRAL 0.05f
#define PID_DERIVATIVE 0.1f
#define INTEGRAL_MAX 1000.0f
#define MAX_DYNAMIC_ERROR 10 // in

/** 
 * @brief Time delay between each temperature correction in ms.
 */
#define TEMP_CORRECTION_INTERVAL (uint16_t)200

/** 
 * @brief Max PWM value for the heater control timer.
 */
#define MAX_PWM_VALUE 1000

/** 
 * @brief I2C address and parameters for VL53L0X time-of-flight sensor.
 */
#define VL53L0X_I2C_ADDR 0x52 
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
