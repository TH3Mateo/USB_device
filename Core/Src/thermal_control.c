//
// Created by M on 25/10/2023.
//

#include "thermal_control.h"
#include "math.h"
#include "configurables.h"

float logR2, R2;

int out = 0;
#define c1 1.009249522e-03
#define c2 2.378405444e-04
#define c3 2.019202697e-07


float calc_temp(uint32_t adc)
{
    double T;
    R2 = R1 * (4096.0 / (float)adc - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2)) - 273.15;
    return T;
}

//uint16_t calc_dac_value(float error,float integral, float derivative,float target_temp){
//    //calculating output with PID formula
////    double dac_out = PID_PROPORTIONAL * error + PID_INTEGRAL * integral + PID_DERIVATIVE * derivative;
//    error = desired_value – actual_value
//            integral = integral_prior + error * iteration_time
//    derivative = (error – error_prior) / iteration_time
//            output = KP*error + KI*integral + KD*derivative + bias
//    error_prior = error
//    integral_prior = integral
//    double dac_out = PID_PROPORTIONAL * error + (PID_INTEGRAL * integral) + (PID_DERIVATIVE * derivative);
//
//
//    return (uint16_t)dac_out;
//};

