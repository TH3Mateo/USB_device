//
// Created by M on 27/07/2023.
//

#include "utils.h"
#include <stdio.h>

const unsigned char* HexToBin(uint8_t byte, unsigned char* bits){


    // Extract the bits
    for (int i = 0; i < 8; i++) {
// Mask each bit in the byte and store it
        bits[i] = (byte >> i) & 1;
    }
    return bits;
// For debug purposes, lets print the received data

}
