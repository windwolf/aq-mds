//
// Created by zhouj on 2024/1/25.
//

#ifndef AQ_DCD24_APP_CONSTANTS_HPP_
#define AQ_DCD24_APP_CONSTANTS_HPP_

#include "config.h"

#define SYSTICK_FREQ_HZ 1000
#define TIM1_PERIOD     (1000000 / PWM_FREQ_HZ)

#define SOFT_ON_DURATION_TICK  (SOFT_ON_DURATION_MS * SYSTICK_FREQ_HZ / 1000)
#define SOFT_OFF_DURATION_TICK (SOFT_OFF_DURATION_MS * SYSTICK_FREQ_HZ / 1000)
#define SOFT_ON_DUTY_INC_STEP  (DUTY_MAX / SOFT_ON_DURATION_TICK);
#define SOFT_OFF_DUTY_DEC_STEP (DUTY_MAX / SOFT_OFF_DURATION_TICK);

#define ADC_MAX_VALUE 255
#define OCP_VALUE \
    (uint16_t)(ADC_MAX_VALUE * OCP_THRESHOLD_A * SHUNT_RES_OHM * CURR_OPA_GAIN / VREF_VOLT)

#define WORK_UNDERLOAD_VALUE \
    (uint16_t)(ADC_MAX_VALUE * WORK_UNDERLOAD_CURRENT_A * SHUNT_RES_OHM * CURR_OPA_GAIN / VREF_VOLT)

/* #define OPP_VALUE                                                                         \
    (uint16_t)(((OPP_MAX_CURR_MA - OPP_MIN_CURR_MA) * (OPP_THRESHOLD_KPA - OPP_MIN_KPA) / \
                    (OPP_MAX_KPA - OPP_MIN_KPA) +                                         \
                OPP_MIN_CURR_MA) *                                                        \
               OPP_SHUNT_RES_OHM * ADC_MAX_VALUE / VREF_VOLT)
*/
#define WORK_TO_IDLE_VALUE                                                                \
    (uint16_t)(OPP_WORK_TO_IDLE_THRESHOLD_MA * OPP_SHUNT_RES_OHM * ADC_MAX_VALUE / 1000 / \
               VREF_VOLT)  // 151
#define IDLE_TO_WORK_VALUE                                                                \
    (uint16_t)(OPP_IDLE_TO_WORK_THRESHOLD_MA * OPP_SHUNT_RES_OHM * ADC_MAX_VALUE / 1000 / \
               VREF_VOLT)  // 99
#define OTP_VALUE              \
    (uint16_t)(ADC_MAX_VALUE * \
               ((OTP_NTC_THRESHOLD_CELSIUS - T0_CELSIUS) * DV_OVER_DT_VC + V0_VOLT) / VREF_VOLT)

#endif  //AQ_DCD24_APP_CONSTANTS_HPP_
