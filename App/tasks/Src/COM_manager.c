#include "COM_manager.h"
#include "configurables.h"

#include "gpio.h"

#include "SEGGER_RTT_printf.h"

osThreadId_t COM_manager_handle;
const osThreadAttr_t COM_attributes = {
        .name = "COM",
        .stack_size = 128 * 2,
        .priority = (osPriority_t) osPriorityAboveNormal1,
};


void COM_manager(void*arguments) {
    struct COM_args *args = (struct COM_args *)arguments;
    MessageBufferHandle_t *receive_queue = args->receive_queue;
    MessageBufferHandle_t *send_queue = args->send_queue;
    uint8_t *connected = args->connected;
    uint32_t retry_timeout = 0;
    uint8_t SendBuffer[RX_BUFF_SIZE] = {0};
    uint8_t sth_to_send = 0;
    USBD_StatusTypeDef send_status = USBD_BUSY;

    // RTT(0,"COM manager started \r \n");

    while (1){
        if(&connected) {
            if (hUsbDeviceFS.dev_connection_status == 0x01){
                xMessageBufferSend(*send_queue, UserRxBufferFS, RX_BUFF_SIZE, portMAX_DELAY);
                hUsbDeviceFS.dev_connection_status = 0x00;
            }
            sth_to_send = ~(uint8_t)xMessageBufferIsEmpty(*send_queue);
            if (sth_to_send == 1) {
                size_t bytes_read = xMessageBufferReceive(*send_queue, SendBuffer, RX_BUFF_SIZE, 0);
                if (bytes_read > 0) {
                    while (send_status!= USBD_OK) {
                    send_status = CDC_Transmit_FS(SendBuffer, bytes_read);
                    }
                send_status = USBD_BUSY; // Reset status for next transmission
                }
            }


            } else {
                if(HAL_GetTick() - retry_timeout > RECONNECTION_TIMEOUT) {
                    RTT(0,"checking for reconnection\r \n");
                        USBD_StatusTypeDef x = USBD_LL_DevConnected(&hUsbDeviceFS);
                        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
                            *connected = 1;
                            RTT(0,"connected \r \n");
                            HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, GPIO_PIN_RESET);
                        } else {
                            retry_timeout = HAL_GetTick();
                        }
                    
            }
            else {
                RTT(0,"waiting for connection \r \n");
                HAL_GPIO_TogglePin(BLUE_LED.port, BLUE_LED.pin);
                HAL_Delay(100);
            }
        }
        }
}