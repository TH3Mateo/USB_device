/**
 * @file        TEMP_manager.c
 * @brief       Source file for the temperature control manager task using FreeRTOS.
 *              Reads temperature from ADC, calculates error, and adjusts PWM output
 *              to control heating based on target temperature using basic control logic.
 * 
 * @authors     Mateusz Turycz  
 *              Aleksander Uliczny
 * 
 * @date        2025-06-02
 * @version     1.0
 */

#include "TEMP_manager.h"
#include "SEGGER_RTT_printf.h"
#include "utils.h"
#include "tim.h"
#include "adc.h"
#include "configurables.h"
#include "thermal_control.h"

/**
 * @brief Handle for the TEMP manager thread.
 */
osThreadId_t TEMP_manager_handle;

/**
 * @brief Thread attributes for the TEMP manager.
 */
const osThreadAttr_t TEMP_attributes = {
    .name = "TEMP",
    .stack_size = 128 * 1,
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * @brief FreeRTOS task for managing temperature measurement and control.
 * 
 * Reads the current temperature via ADC, compares it to the target,
 * and adjusts PWM output using a simple proportional-based control logic.
 * 
 * @param arguments Pointer to a @ref TEMP_args structure containing mutex-protected
 *                  temperature values and heater state.
 */
void TEMP_manager(void *arguments)
{
    struct TEMP_args *args = (struct TEMP_args *)arguments;
    MUTEX_f *out_temp = args->CURRENT_TEMP;
    MUTEX_f *wanted_temp = args->TARGET_TEMP;
    MUTEX_uint8 *heater_state = args->heater_state;

    HAL_TIM_PWM_Init(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100 - 0);

    HAL_ADC_Start(&hadc1);
    double TARGET_TEMP = 0.0;
    double CURRENT_TEMP = 0.0;

    double error = 0.0, prev_error = 0.0;
    double integral = 0.0;
    double derivative = 0.0;
    double dac_out = 0.0;
    double new_temp = 0.0;

    uint32_t last_correction_time = HAL_GetTick();

    while (1)
    {
        if (HAL_GetTick() - last_correction_time >= TEMP_CORRECTION_INTERVAL)
        {
            while (xSemaphoreTake(wanted_temp->semaphore, pdMS_TO_TICKS(10)) != pdTRUE)
                ;

            TARGET_TEMP = wanted_temp->value;
            xSemaphoreGive(wanted_temp->semaphore);

            last_correction_time = HAL_GetTick();

            // Read temperature from ADC
            HAL_ADC_Start(&hadc1);
            vTaskDelay(10); // Replace with non-blocking ADC later if needed
            new_temp = 0.0;
            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
            {
                new_temp = calc_temp(HAL_ADC_GetValue(&hadc1));
            }
            HAL_ADC_Stop(&hadc1);

            // PID computation
            error = TARGET_TEMP == 0.0 ? 0.0 : (TARGET_TEMP - new_temp);
            integral += error * TEMP_CORRECTION_INTERVAL;
            if (integral > INTEGRAL_MAX)
                integral = INTEGRAL_MAX;
            else if (integral < -INTEGRAL_MAX)
                integral = -INTEGRAL_MAX;
            derivative = (error - prev_error) / TEMP_CORRECTION_INTERVAL;

            dac_out = PID_PROPORTIONAL * error + PID_INTEGRAL * integral + PID_DERIVATIVE * derivative;

            // Clamp dac_out to 0–100 range
            if (dac_out > 100.0)
                dac_out = 100.0;
            if (dac_out < 0.0)
                dac_out = 0.0;

            // Update PWM output
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dac_out);

            // Save current values at the end
            CURRENT_TEMP = new_temp;
            prev_error = error;

            while (xSemaphoreTake(out_temp->semaphore, pdMS_TO_TICKS(10)) != pdTRUE)
                ;

            out_temp->value = CURRENT_TEMP;
            xSemaphoreGive(out_temp->semaphore);

            while (xSemaphoreTake(heater_state->semaphore, pdMS_TO_TICKS(10)) != pdTRUE)
                ;
            heater_state->value = (dac_out > 0.0) ? 1 : 0; // 1 for ON, 0 for OFF
            xSemaphoreGive(heater_state->semaphore);
        }
    }
}
