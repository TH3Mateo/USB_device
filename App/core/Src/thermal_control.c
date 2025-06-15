/**
 * @file        thermal_control.c
 * @brief       Implements temperature calculation and PID control for thermal regulation.
 *
 * @details     Provides functions to:
 *              - Convert ADC values from an NTC thermistor to Celsius degrees
 *              - Calculate DAC output for heater control using a PID algorithm
 *
 * @authors     Mateusz Turycz
 *              Aleksander Uliczny
 * @date        2025-05-21
 */

#include "thermal_control.h"
#include "math.h"
#include "configurables.h"

float logR2, R2;
int out = 0;

#define BETA 3950.0    ///< Beta coefficient of the thermistor
#define T_25 298.15    ///< Reference temperature in Kelvin (25°C)
#define R1 96600.0     ///< Series resistor value in ohms
#define R_T25 100000.0 ///< Resistance of thermistor at 25°C

double T;

/**
 * @brief Rounds a double to two decimal places.
 *
 * @param value The input value.
 * @return Rounded double value.
 */
double round_to_two_decimal_places(double value)
{
    return round(value * 100.0) / 100.0;
}

/**
 * @brief Calculates temperature in Celsius from ADC reading of NTC thermistor.
 *
 * @param adc Raw ADC value (0–4095).
 * @return Temperature in Celsius, rounded to two decimal places.
 */
double calc_temp(uint32_t adc)
{
    float adc_voltage = (float)adc * 3.3 / 4095;
    double r_ntc = R1 * (adc / 4096.0) / (1 - (adc / 4096.0));

    // Calculate temperature using Beta formula
    double temp_kelvin = BETA / (log(r_ntc / R_T25) + (BETA / T_25));
    return round_to_two_decimal_places(temp_kelvin - 273.15); // Convert to Celsius
}

/**
 * @brief Calculates DAC output value using a PID control algorithm.
 *
 * Uses the formula:
 * output = Kp * error + Ki * integral + Kd * derivative
 *
 * @param error Current temperature error.
 * @param integral Integral of past errors.
 * @param derivative Derivative (rate of change) of error.
 * @return DAC output value as a 16-bit unsigned integer.
 */
uint16_t calc_dac_value(float error, float integral, float derivative)
{
    double dac_out = PID_PROPORTIONAL * error +
                     PID_INTEGRAL * integral +
                     PID_DERIVATIVE * derivative;

    return (uint16_t)dac_out;
}
