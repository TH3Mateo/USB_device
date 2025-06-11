#pragma once
#include "FreeRTOS.h"
#include "cmsis_os2.h"

void DISPLAY_manager(void *argument);

extern osThreadId_t DISPLAY_manager_handle;
extern const osThreadAttr_t DISPLAY_attributes;