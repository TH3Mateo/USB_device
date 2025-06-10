//
// Created by M on 14/05/2025.
//

#include "utils.h"
#include <stdio.h>
#include <math.h>

const unsigned char* HexToBin(uint8_t byte, unsigned char* bits){


    // Extract the bits
    for (int i = 0; i < 8; i++) {
// Mask each bit in the byte and store it
        bits[i] = (byte >> i) & 1;
    }
    return bits;
// For debug purposes, lets print the received data

}

float HexToDec(uint8_t* hex, uint8_t byte_count){
    int dec = 0;
    for(int i = 0; i < byte_count; i++) {
        dec=dec+(hex[i]<<8*(byte_count-i-1));
    }
    return dec/pow(2,16);
}
