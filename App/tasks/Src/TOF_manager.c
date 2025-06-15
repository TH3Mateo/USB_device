/**
 * @file        TOF_manager.c
 * @brief       Source file for the Time-of-Flight sensor manager task using FreeRTOS.
 *              Sets up VL53L1X sensor and includes placeholders for configuration
 *              and future ranging loop logic.
 *
 * @authors     Mateusz Turycz
 *              Aleksander Uliczny
 *
 * @date        2025-05-24
 * @version     1.0
 */

#pragma once

#include "TOF_manager.h"
#include "SEGGER_RTT_printf.h"
#include "vl53l1_api.h"
#include "i2c.h"

/**
 * @brief Handle for the TOF manager thread.
 */
osThreadId_t TOF_manager_handle;

/**
 * @brief Thread attributes for the TOF manager thread.
 */
const osThreadAttr_t TOF_attributes = {
	.name = "TOF",
	.stack_size = 128 * 1,
	.priority = (osPriority_t)osPriorityNormal,
};

#define isInterrupt 0 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

/**
 * @brief FreeRTOS task for initializing and managing a VL53L1X TOF sensor.
 *
 * Prepares the sensor for ranging by setting its I2C address and configuration,
 * with placeholders for future implementation of the ranging loop.
 *
 * @param argument Pointer to @ref TOF_args structure containing a mutex-protected distance value.
 */
void TOF_manager(void *argument)
{
	struct TOF_args *args = (struct TOF_args *)argument;
	MUTEX_f distance = *args->distance;

	uint8_t status, loop;
	uint8_t addr = 0x52;
	uint16_t sensor_id;

	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Default VL53L1X Ultra Low Power I2C address */

	uint8_t state = 1;

	VL53L1_Dev_t dev;
	VL53L1_DEV Dev = &dev;
	VL53L1_RangingMeasurementData_t RangingData;
	Dev->I2cHandle = &hi2c1;
	Dev->I2cDevAddr = 0x52;

	uint8_t ToFSensor = 1; // Select ToFSensor: 0=Left, 1=Center, 2=Right
						   // status = XNUCLEO53L1A1_ResetId(ToFSensor, 0); // Reset ToF sensor
						   // HAL_Delay(2);
						   // status = XNUCLEO53L1A1_ResetId(ToFSensor, 1); // Reset ToF sensor
						   // HAL_Delay(2);

	uint8_t byteData;
	uint16_t wordData;
	volatile int IntCount;

	uint8_t firstTimeInterrupt = 1;
	printf("Fast Ranging Test\n");
	status = VL53L1_WaitDeviceBooted(Dev);
	status = VL53L1_DataInit(Dev);
	status = VL53L1_StaticInit(Dev);
	status = VL53L1_SetPresetMode(Dev, VL53L1_PRESETMODE_LITE_RANGING);
	status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
	status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 10000);
	vTaskDelay(30); // Wait for the sensor to be ready
	for (int attempt = 0; attempt < 5; attempt++)
	{
		status = VL53L1_StartMeasurement(Dev);
		if (status == 0)
		{
			break;
		}
		vTaskDelay(10);
	}

	do /* polling mode */
	{
		status = VL53L1_WaitMeasurementDataReady(Dev);
		if (!status)
		{
			if (firstTimeInterrupt == 0)
			{
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if (status == 0)
				{
					printf("Distance: %d\n", RangingData.RangeMilliMeter);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
			else
			{ /* Must not read data at the first interrupt, must clear interrupt and start measurement */
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
				firstTimeInterrupt = 0;
			}
			while (!xSemaphoreTake(distance.semaphore, portMAX_DELAY))
				;
			distance.value = RangingData.RangeMilliMeter;
			xSemaphoreGive(distance.semaphore);
			vTaskDelay(200);
		}

	} while (1);

	//  return status;

	RTT(0, "End of VL53L1X ultra low power demo\n");
}
