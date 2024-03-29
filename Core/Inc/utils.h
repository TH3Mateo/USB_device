//
// Created by M on 27/07/2023.
//

#ifndef USB_UTILS_H
#define USB_UTILS_H
#include "stdint.h"

const unsigned char* HexToBin(uint8_t byte, unsigned char* bits);

float HexToDec(uint8_t* hex, uint8_t byte_count);
#endif //USB_UTILS_H
