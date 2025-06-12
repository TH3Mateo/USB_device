#include "COM_manager.h"
#include "configurables.h"

#include "gpio.h"

#include "SEGGER_RTT_printf.h"

osThreadId_t COM_manager_handle;
const osThreadAttr_t COM_attributes = {
    .name = "COM",
    .stack_size = 128 * 2,
    .priority = (osPriority_t)osPriorityNormal,
};

void COM_manager(void *arguments)
{
    MX_USB_DEVICE_Init(); // Initialize USB device

    struct COM_args *args = (struct COM_args *)arguments;
    MessageBufferHandle_t receive_queue = *(args->receive_queue);
    MessageBufferHandle_t send_queue = *(args->send_queue);
    MUTEX_uint8 *connected = args->connected; // pointer to struct

    uint32_t retry_timeout = 0;
    uint8_t SendBuffer[RX_BUFF_SIZE] = {0};
    USBD_StatusTypeDef send_status = USBD_BUSY;

    while (1)
    {
        uint8_t is_connected = 0;
        // Take mutex to read connected flag safely
        if (xSemaphoreTake(connected->semaphore, portMAX_DELAY) == pdTRUE)
        {
            is_connected = connected->value;
            xSemaphoreGive(connected->semaphore);
        }

        if (is_connected)
        {
            if (hUsbDeviceFS.dev_connection_status == 0x01)
            {
                RTT(0, "received data \r\n");
                RTT(0, "dev_state: %d \r\n", hUsbDeviceFS.dev_state);
                RTT(0, "command: %d \r\n", UserRxBufferFS[1]);

                // Send received data to send_queue buffer
                xMessageBufferSend(receive_queue, UserRxBufferFS, RX_BUFF_SIZE, portMAX_DELAY);

                hUsbDeviceFS.dev_connection_status = 0x00;
            }

            // Check if send_queue has data to send
            if (xMessageBufferIsEmpty(send_queue) == pdFALSE)
            {
                size_t bytes_read = xMessageBufferReceive(send_queue, SendBuffer, RX_BUFF_SIZE, 0);
                if (bytes_read > 0)
                {
                    // Transmit until successful
                    do
                    {
                        send_status = CDC_Transmit_FS(SendBuffer, bytes_read);
                    } while (send_status != USBD_OK);
                    send_status = USBD_BUSY; // Reset for next transmission
                }
            }
        }
        else
        {
            if ((HAL_GetTick() - retry_timeout) > RECONNECTION_TIMEOUT)
            {
                RTT(0, "checking for reconnection\r\n");
                USBD_StatusTypeDef x = USBD_LL_DevConnected(&hUsbDeviceFS); // This call seems unused, consider checking its return

                if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
                {
                    // Set connected flag safely
                    if (xSemaphoreTake(connected->semaphore, portMAX_DELAY) == pdTRUE)
                    {
                        connected->value = 1;
                        xSemaphoreGive(connected->semaphore);
                    }
                    RTT(0, "connected \r\n");
                    HAL_GPIO_WritePin(BLUE_LED.port, BLUE_LED.pin, GPIO_PIN_SET);
                }
                else
                {
                    retry_timeout = HAL_GetTick();
                }
            }
            else
            {
                RTT(0, "waiting for connection \r\n");
                HAL_GPIO_TogglePin(BLUE_LED.port, BLUE_LED.pin);
                HAL_Delay(100);
            }
        }
        // Optional short delay to prevent task hogging CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
