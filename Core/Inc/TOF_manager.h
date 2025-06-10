#pragma once
#include "FreeRTOS.h"
#include "cmsis_os2.h"

void TOF_manager(void *argument);


extern osThreadId_t TOF_manager_handle;
extern const osThreadAttr_t TOF_attributes;

