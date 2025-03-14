#ifndef ___CONFIG_H__
#define ___CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define APP_NAME    "Wibotlib4arm_project_template"
#define APP_VERSION "1.0"

#define MOTOR_PWM_FREQ_HZ          10000
#define MOTOR_SOFT_ON_DURATION_MS  1000
#define MOTOR_SOFT_OFF_DURATION_MS 1000

#define OVER_CURRENT_THRESHOLD_AMP  150
#define OVER_VOLTAGE_THRESHOLD_VOLT 80
#define OVER_TEMP_THRESHOLD_CELSIUS 100

/* BASE ON HW */
#define VREF_VOLT     (3.3)
#define SHUNT_RES_OHM (0.001)
#define CURR_OPA_GAIN (11.0)

#define VBUS_DIV_GAIN (11 / (100.0 + 11))

#define V0_VOLT       (1.055)
#define T0_CELSIUS    (25.0)
#define DV_OVER_DT_VC (0.0249)

#define ERROR_CD_COUNT 5000  // in ISR tick

#ifdef __cplusplus
}
#endif

#endif  // ___CONFIG_H__
