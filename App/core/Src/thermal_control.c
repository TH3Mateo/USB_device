//
// Created by M on 25/10/2023.
//

#include "thermal_control.h"
#include "math.h"
#include "configurables.h"

float logR2, R2;

int out = 0;
#define BETA 3950.0
#define T_25 298.15
#define R1 96600.0
#define R_T25 100000.0


double T;

double round_to_two_decimal_places(double value) {
    return round(value * 100.0) / 100.0;
}
double calc_temp(uint32_t adc)
{
    float adc_voltage = (float)adc * 3.3 / 4095;
    double r_ntc = R1 * (adc/4096.0)/(1-(adc/4096.0));

    // Calculate temperature using Beta formula
    double temp_kelvin = BETA / (log(r_ntc / R_T25) + (BETA / T_25));
    return round_to_two_decimal_places( temp_kelvin - 273.15);  // Convert to Celsius

}

uint16_t calc_dac_value(float error,float integral, float derivative){
    //calculating output with PID formula
    double dac_out = PID_PROPORTIONAL * error + PID_INTEGRAL * integral + PID_DERIVATIVE * derivative;




    return (uint16_t)dac_out;
};

