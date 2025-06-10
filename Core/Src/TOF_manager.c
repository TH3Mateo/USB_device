#pragma once

#include "TOF_manager.h"
#include "VL53L1X_ULP_api.h"
#include "SEGGER_RTT_printf.h"

osThreadId_t TOF_manager_handle;

const osThreadAttr_t TOF_attributes = {
        .name = "TOF",
        .stack_size = 128 * 1,
        .priority = (osPriority_t) osPriorityNormal,
};

void TOF_manager(void *argument) {
    RTT(0,"TOF manager started \r \n");
    char msgBuffer[52];
  	uint8_t 				status, loop;
	uint8_t 				dev;
	uint16_t 				sensor_id;

	dev = 0x52;
	for (uint8_t i = 0; i < 52; i++) {
		msgBuffer[i] = ' ';
	}

	status = VL53L1X_ULP_GetSensorId(dev, &sensor_id);
	if(status || (sensor_id != 0xEACC))
	{
		RTT(0,"VL53L1X not detected at requested address\n");
	}

}