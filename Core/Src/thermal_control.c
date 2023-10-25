//
// Created by M on 25/10/2023.
//

#include "thermal_control.h"
#include "math.h"
#include "stdio.h"

const int R1 = 10000;
float logR2, R2;

int out = 0;
#define c1 1.009249522e-03
#define c2 2.378405444e-04
#define c3 2.019202697e-07

int map_f(double x, long in_min, long in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double calc_temp(float V)
{
    double T;
    R2 = R1 * (1023.0 / (float)V - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273.15;
    return T;
}

