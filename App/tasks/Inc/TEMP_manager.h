// #pragma once
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "utils.h"

extern osThreadId_t TEMP_manager_handle;
extern const osThreadAttr_t TEMP_attributes;
struct TEMP_args {MUTEX_f *CURRENT_TEMP;MUTEX_f *TARGET_TEMP;MUTEX_uint8 *heater_state;};

void TEMP_manager(void* arguments);

