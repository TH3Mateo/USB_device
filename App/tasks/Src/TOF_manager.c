#pragma once

#include "TOF_manager.h"
#include "SEGGER_RTT_printf.h"

osThreadId_t TOF_manager_handle;


const osThreadAttr_t TOF_attributes = {
        .name = "TOF",
        .stack_size = 128 * 1,
        .priority = (osPriority_t) osPriorityNormal,
};




void TOF_manager(void *argument) {
	struct TOF_args *args = (struct TOF_args *)argument;
	MUTEX_f distance = *args->distance;

	uint8_t 				status, loop;
	uint8_t 				dev;
	uint16_t 				sensor_id;


	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Default VL53L1X Ultra Low Power I2C address */
	dev = 0x52;
	uint8_t state =1;

	while(state!=0){

		HAL_Delay(100);
	}

	while(1){
		
	}
	/* (Optional) Change I2C address */
	// status = VL53L1X_ULP_SetI2CAddress(dev, 0x20);
	// dev = 0x20;


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L1X sensor connected */




	/*********************************/
	/*     Sensor configuration      */
	/*********************************/

	/* (Optional) Program sensor to raise an interrupt ONLY below 300mm */

	/* (Optional) Program a 10Hz ranging frequency */


	/*********************************/
	/*         Ranging loop          */
	/*********************************/



	RTT(0,"End of VL53L1X ultra low power demo\n");
}