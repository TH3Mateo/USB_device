#include "DISPLAY_manager.h"
#include "SEGGER_RTT_printf.h"




const osThreadAttr_t DISPLAY_attributes = {
        .name = "DISPLAY",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};


void DISPLAY_manager(void *argument) {
    RTT(0,"DISPLAY manager started \r \n");
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      initLCD();
  char *stl = "Github: @fatay";
  writeLCD(stl);
  char *ttl = "LCD Library";
  setCursor(0,1);

  writeLCD(ttl);
  
    while (1) {
        osDelay(400);
    }
}
