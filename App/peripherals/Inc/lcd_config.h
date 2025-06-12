#pragma once
#include "stm32f4xx.h"

#define LCD_4BIT 1
#define LCD_8BIT 2
// Choose a microcontroller family
// #define STM32F0
// #define STM32F1
// #define STM32F3
#define STM32F4
// #define STM32F7

#define MODE LCD_8BIT

/* CONFIG FOR LIBRARY USER */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO;

// 4 pin mode -> pins

GPIO DATA6_Pin = {GPIOA, GPIO_PIN_7};
GPIO DATA7_Pin = {GPIOB, GPIO_PIN_0};
GPIO DATA8_Pin = {GPIOB, GPIO_PIN_1};
GPIO DATA5_Pin = {GPIOB, GPIO_PIN_2};

GPIO RS_Pin = {.port = GPIOB, .pin = GPIO_PIN_13};
GPIO EN_Pin = {.port = GPIOB, .pin = GPIO_PIN_15};
// RW Pin not used,connect to GND

// if you want to work with 8 bit mode uncomment the area which is given below

#if MODE == LCD_8BIT

GPIO DATA1_Pin = {.port = GPIOA, .pin = GPIO_PIN_3};
GPIO DATA2_Pin = {.port = GPIOA, .pin = GPIO_PIN_4};
GPIO DATA3_Pin = {.port = GPIOA, .pin = GPIO_PIN_5};
GPIO DATA4_Pin = {.port = GPIOA, .pin = GPIO_PIN_6};
#endif

// #endif /* INC_LCD_CONFIG_H_ */
