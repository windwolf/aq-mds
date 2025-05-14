#ifndef ___CONFIG_H__
#define ___CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define APP_NAME    "AQ-MDS24"
#define APP_VERSION "1.0.17"

#define PWM_FREQ_HZ          10000
#define SOFT_ON_DURATION_MS  1000
#define SOFT_OFF_DURATION_MS 1000

// 驱动过流阈值
#define OCP_THRESHOLD_A 180

// 驱动过温阈值
#define OTP_NTC_THRESHOLD_CELSIUS 100

// 工作压力阈值
#define OPP_WORK_TO_IDLE_THRESHOLD_MA 13.5  //  mA
#define OPP_IDLE_TO_WORK_THRESHOLD_MA 8.5   // mA
// #define OPP_THRESHOLD_MA 10  // 10 mA

// 空闲检测时间
#define IDLE_TIMEOUT_MS 6000

#define IDLE_SLEEP_TIMEOUT_MS 1500

#define WORK_UNDERLOAD_CURRENT_A 20

// 节能模式占空比
#define IDEL_DUTY 300
#define DUTY_MAX  1000

// --- BASE ON HW ------------------
#define VREF_VOLT     (3.3)
#define SHUNT_RES_OHM (0.001)
#define CURR_OPA_GAIN (11.0)

// ADC
#define ADC_IDX_OPP1 0
#define ADC_IDX_OPP2 1
#define ADC_IDX_NTC  2
#define ADC_IDX_OCP  3

// NTC
#define V0_VOLT       (1.055)
#define T0_CELSIUS    (25.0)
#define DV_OVER_DT_VC (0.0249)

// OPP
//#define OPP_MAX_KPA       (6000)
//#define OPP_MIN_KPA       (0)
#define OPP_MAX_CURR_MA   (20)
#define OPP_MIN_CURR_MA   (4)
#define OPP_SHUNT_RES_OHM (150)

#define ERROR_CD_COUNT (5000)  // in ISR tick

#ifdef __cplusplus
}
#endif

#endif  // ___CONFIG_H__
