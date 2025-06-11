//
// Created by M on 25/10/2023.
//

#ifndef USB_THERMAL_CONTROL_H
#define USB_THERMAL_CONTROL_H

#include <stdint.h>

double calc_temp(uint32_t V);

uint16_t calc_dac_value(float error,float integral, float derivative);

#endif //USB_THERMAL_CONTROL_H
