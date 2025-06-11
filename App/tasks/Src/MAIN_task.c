#include "MAIN_task.h"
#include "commands.h"
#include "gpio.h"

extern MessageBufferHandle_t *send_queue;

void MAIN_task(void* args){
    int result = parse_packet(UserRxBufferFS, RX_BUFF_SIZE, &cmd, payload, &len);

            if (result == 0) {
                switch (cmd) {
                    case SET_LED_STATE: {
                        if (len < 2) break;
                        uint8_t led_nr = payload[0];
                        uint8_t state = payload[1];

                        const char *msg;
                        if (led_nr == 0x01) {
                            HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, !state);
                            msg = "BL switched";
                        } else if (led_nr == 0x02) {
                            HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, !state);
                            msg = "EL switched";
                        } else {
                            msg = "unknown LED";
                        }

                        create_packet(response, SET_LED_STATE, (uint8_t *)msg, strlen(msg));
                        CDC_Transmit_FS(response, 4 + strlen(msg));
                        break;
                    }

                    case REQUEST_CURRENT_TEMPERATURE: {
                        double temp = ACTUAL_TEMP.value;
                        create_packet(response, REQUEST_CURRENT_TEMPERATURE, (uint8_t *)&temp, sizeof(temp));
                        // while(status != USBD_OK){
                        //   status = CDC_Transmit_FS(response, 4 + sizeof(temp));
                        // }
                        xMessageBufferSend(*send_queue, response, 4 + sizeof(temp), portMAX_DELAY);
                        status = USBD_BUSY; 
                        break;
                    }

                    case SET_TARGET_TEMPERATURE: {
                        if (len < sizeof(float)) break;
                        float target;
                        memcpy(&target, payload, sizeof(float));
                        TARGET_TEMP.value = target;

                        create_packet(response, SET_TARGET_TEMPERATURE, (uint8_t *)&target, sizeof(target));
                        // while(status != USBD_OK){
                        //   status = CDC_Transmit_FS(response, 4 + sizeof(target));
                        // }
                        status = USBD_BUSY; 
                        break;
                    }

                    case REQUEST_ACTUAL_HEATER_STATE: {
                        uint8_t heater_state = 25; // przykładowa wartość
                        create_packet(response, REQUEST_ACTUAL_HEATER_STATE, &heater_state, 1);
                        // while(status != USBD_OK){
                        //   status = CDC_Transmit_FS(response, 5);
                        // }
                        status = USBD_BUSY; 
                        break;
                    }

                    default: {
                        const char *err = "Unknown CMD";
                        create_packet(response, 0xFF, (uint8_t *)err, strlen(err));
                        // CDC_Transmit_FS(response, 4 + strlen(err));
                        break;
                    }
                }
            } else {
                printf("Bad packet (err %d)\r\n", result);
            }
}