#pragma once
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "utils.h"
#include "lcd.h"


extern osThreadId_t DISPLAY_manager_handle;

extern const osThreadAttr_t DISPLAY_attributes;
struct DISPLAY_args
{
    MUTEX_f *CURRENT_TEMP;
    MUTEX_f *TARGET_TEMP;
    MUTEX_uint8 *heater_state;
    MUTEX_uint8 *USB_connected;
};

void DISPLAY_manager(void *argument);