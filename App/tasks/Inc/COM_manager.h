#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "utils.h"
#include "message_buffer.h"

#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

extern osThreadId_t COM_manager_handle;
extern const osThreadAttr_t COM_attributes;
struct COM_args{
    MessageBufferHandle_t* receive_queue;
    MessageBufferHandle_t* send_queue;
    MUTEX_uint8 *connected;
};


void COM_manager(void *args);
