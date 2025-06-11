#include "TEMP_manager.h"
#include "SEGGER_RTT_printf.h"
#include "utils.h"
#include "tim.h"
#include "adc.h"
#include "configurables.h"
#include "thermal_control.h"


osThreadId_t TEMP_manager_handle;
const osThreadAttr_t TEMP_attributes = {
        .name = "TEMP",
        .stack_size = 128 * 1,
        .priority = (osPriority_t) osPriorityNormal,
};

void TEMP_manager(void* arguments) {
    struct TEMP_args *args = (struct TEMP_args *)arguments;
    MUTEX_f *CURRENT_TEMP = args->CURRENT_TEMP;
    MUTEX_f *TARGET_TEMP = args->TARGET_TEMP;
    MUTEX_uint8 *heater_state = args->heater_state;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 100-0);

    HAL_ADC_Start(&hadc1);
    uint16_t dac_out;
    float prev_error = 0;
    float integral = 0;
    float prev_integral = 0;
    float error = 0;
    uint32_t time_variable=HAL_GetTick();
//    uint32_t time_diff;
    while (1) {


        if(HAL_GetTick()%TEMP_CORRECTION_INTERVAL==0) {
//            xSemaphoreTake(CURRENT_TEMP.semaphore, portMAX_DELAY);
            HAL_ADC_Start(&hadc1);
            HAL_Delay(10);
            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
                CURRENT_TEMP->value = calc_temp(HAL_ADC_GetValue(&hadc1));

            }
            HAL_ADC_Stop(&hadc1);

            time_variable = HAL_GetTick()-time_variable;
            error = TARGET_TEMP->value==0 ? 0.0 : (TARGET_TEMP->value - CURRENT_TEMP->value);
            integral = prev_integral + error * TEMP_CORRECTION_INTERVAL;

//            dac_out = PID_PROPORTIONAL*error + PID_INTEGRAL*integral + PID_DERIVATIVE*((error - prev_error ) / TEMP_CORRECTION_INTERVAL);
            dac_out =  (TARGET_TEMP->value   - CURRENT_TEMP->value)*100/CURRENT_TEMP->value;
            prev_error = error;
            prev_integral = integral;
            time_variable = HAL_GetTick();

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dac_out > 100 ? 0:(100-dac_out));




        }
    }
}