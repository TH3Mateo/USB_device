#pragma once
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "utils.h"


extern osThreadId_t TOF_manager_handle;
extern const osThreadAttr_t TOF_attributes;

struct TOF_args {
    MUTEX_f *distance;
    // Add other parameters if needed
};

void TOF_manager(void *argument);
