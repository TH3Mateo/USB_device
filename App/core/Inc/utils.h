//
// Created by M on 27/07/2023.
//
#pragma once

#ifndef USB_UTILS_H
#define USB_UTILS_H
#include "stm32f4xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <FreeRTOS.h>
#include <stdint.h>
#include "cmsis_os2.h"
#include "semphr.h"


const unsigned char* HexToBin(uint8_t byte, unsigned char* bits);
float HexToDec(uint8_t* hex, uint8_t byte_count);
uint8_t calculate_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload);

// Tworzy pakiet: [START][CMD][LEN][PAYLOAD][CHECKSUM]
int create_packet(uint8_t *buffer_out, uint8_t cmd, const uint8_t *payload, uint8_t length);

// Parsuje pakiet, sprawdza poprawność i wypisuje dane
int parse_packet(const uint8_t *buffer_in, int buffer_len, uint8_t *cmd_out, uint8_t *payload_out, uint8_t *len_out);

typedef struct {
    SemaphoreHandle_t semaphore;
    double value;
} MUTEX_f;

typedef struct {
    SemaphoreHandle_t semaphore;
    GPIO_TypeDef* port;
    uint16_t pin;
} MUTEX_digitPin;


typedef struct {
    SemaphoreHandle_t semaphore;
    uint8_t value;
} MUTEX_uint8;

#endif //USB_UTILS_H