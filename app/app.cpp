#include <functional>
#include <utility>
#include "constants.hpp"
#include "main.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"
#include "pin.hpp"

#include "logger.hpp"
#include "app-framework.hpp"
#include "io/gpio-switch.hpp"
#include "filter/hysteresis-comparer.hpp"
#include "filter/lp.hpp"

LOGGER("app")

using namespace wibot;

extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

enum class MotorState : uint8_t
{
    IDLE = 0,
    IDLE_SLEEP,
    WORK,
    ERROR,
};
static const char* motorStateStr[] = {
    "IDLE",
    "IDLE_SLEEP",
    "WORK",
    "ERROR",
};

Pin pinOTP(GPI_OTP_GPIO_Port, GPI_OTP_Pin); // true: normal; false:over temp

bool isInitialized = false;

uint16_t adcData[4]; // OPP1, OPP2, NTC, OCP
uint16_t adcDataOffset[4];
uint16_t adcValue[4];

//constexpr static const HysteresisComparerBounds<uint16_t> wp1Bounds[1] = {{
//    .lowBound  = WP_VALUE,
//    .highBound = WP_VALUE + 10,
//}};
FirstOrderLowPassFilterConfig oppFilterConfig = {
    .sampleTime = 0.001f,
    .cutoffFreq = 100.0f,
    .wrapValue = 0.0f,
};

FirstOrderLowPassFilter opp1Filter;
FirstOrderLowPassFilter opp2Filter;

FirstOrderLowPassFilterConfig motorCurrentFilterConfig = {
    .sampleTime = 0.001f,
    .cutoffFreq = 100.0f,
    .wrapValue = 0.0f,
};

FirstOrderLowPassFilter motorCurrentFilter;
//HysteresisComparer wp1Comp(wp1Bounds, 1);
//HysteresisComparer wp2Comp(wp1Bounds, 1);

bool isOverCurrent;
bool isUnderLoad;
bool isWork;
bool isOverTemp;
bool isOverTempOil;

static void calcAdcValueOffset() {
    adcDataOffset[ADC_IDX_OPP1] = 0;
    adcDataOffset[ADC_IDX_OPP2] = 0;
    adcDataOffset[ADC_IDX_NTC] = 0;
    adcDataOffset[ADC_IDX_OCP] = adcData[ADC_IDX_OCP];
}

/**
 * calc adc value.
 * Curr:
 */
static void updateState(MotorState currentState) {
    int32_t opp1 = (int32_t)adcData[ADC_IDX_OPP1] - (int32_t)adcDataOffset[ADC_IDX_OPP1];
    int32_t opp2 = (int32_t)adcData[ADC_IDX_OPP2] - (int32_t)adcDataOffset[ADC_IDX_OPP2];
    int32_t ntc = (int32_t)adcData[ADC_IDX_NTC] - (int32_t)adcDataOffset[ADC_IDX_NTC];
    int32_t ocp = (int32_t)adcData[ADC_IDX_OCP] - (int32_t)adcDataOffset[ADC_IDX_OCP];

    adcValue[ADC_IDX_OPP1] = (opp1 < 0) ? 0 : opp1;
    adcValue[ADC_IDX_OPP2] = (opp2 < 0) ? 0 : opp2;
    adcValue[ADC_IDX_NTC] = (ntc < 0) ? 0 : ntc;
    adcValue[ADC_IDX_OCP] = (ocp < 0) ? 0 : ocp;

    auto opp1Value = (uint16_t)opp1Filter.filter(adcValue[ADC_IDX_OPP1]);
    auto opp2Value = (uint16_t)opp2Filter.filter(adcValue[ADC_IDX_OPP2]);
    auto currentValue = (uint16_t)motorCurrentFilter.filter(adcValue[ADC_IDX_OCP]);

    // OCP 反算公式：ADC值/255*3.3/11/0.001/180
    isOverCurrent = currentValue >= OCP_VALUE; // 1094

    isUnderLoad = currentValue >= WORK_UNDERLOAD_VALUE; // 1094

    // OPP 反算公式：ADC值/255*3.3/150
    if (currentState == MotorState::WORK) {
        isWork = (opp1Value >= WORK_TO_IDLE_VALUE) || (opp2Value >= WORK_TO_IDLE_VALUE);
    }
    else if (currentState == MotorState::IDLE_SLEEP) {
        isWork = (opp1Value >= IDLE_TO_WORK_VALUE) || (opp2Value >= IDLE_TO_WORK_VALUE);
    }
    else if (currentState == MotorState::IDLE) {
        isWork = (opp1Value >= IDLE_TO_WORK_VALUE) || (opp2Value >= IDLE_TO_WORK_VALUE);
    }
    else {
        isWork = (opp1Value >= IDLE_TO_WORK_VALUE) || (opp2Value >= IDLE_TO_WORK_VALUE);
    }
    // NTC 反算公式：(ADC值/255*3.3-1.055)/0.0249+25
    isOverTemp = adcValue[ADC_IDX_NTC] >= OTP_VALUE; // 3008
    isOverTempOil = pinOTP.read() == PinStatus::kReset;
}

static void peripheralInit() {
    HAL_ADC_Stop(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc1);

    opp1Filter.setConfig(oppFilterConfig);
    opp2Filter.setConfig(oppFilterConfig);

    motorCurrentFilter.setConfig(motorCurrentFilterConfig);

    HAL_TIM_PWM_DeInit(&htim1);
    htim1.Init.Period = TIM1_PERIOD - 1;
    HAL_TIM_PWM_Init(&htim1);
    LL_TIM_OC_SetCompareCH1(htim1.Instance, 0);
}

static void peripheralStart() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, 4);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    // HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
}

static inline void setDuty(uint16_t duty) {
    LL_TIM_OC_SetCompareCH2(TIM1, TIM1_PERIOD * duty / 1000);
}

class App : public AppFramework
{
    public:
        App() : AppFramework(APP_NAME, APP_VERSION) {
        }

        void onRun() override {
            peripheralInit();

            HAL_Delay(100);

            peripheralStart();

            HAL_Delay(100);

            calcAdcValueOffset();

            HAL_Delay(100);

            isInitialized = true;

            motorState = MotorState::WORK;

            while (1) {
                updateState(motorState);

                if (isOverTemp) {
                    MotorState
                        motorState = MotorState::ERROR;
                    LOG_I(
                        "state: %s, duty: %d, VOPP1:%d, VOPP2:%d, VNTC:%d, VOCP:%d, WK: %d, OC: "
                        "%d, OTD: %d, "
                        "OTO: %d",
                        motorStateStr[(uint8_t)motorState],
                        _actualDuty,
                        adcValue[ADC_IDX_OPP1],
                        adcValue[ADC_IDX_OPP2],
                        adcValue[ADC_IDX_NTC],
                        adcValue[ADC_IDX_OCP],
                        isWork,
                        isOverCurrent,
                        isOverTemp,
                        isOverTempOil);
                    LOG_I("Entry ERROR state");
                }
                else {
                    if (motorState == MotorState::WORK) {
                        if (isWork || isUnderLoad) {
                            // 电机带载，则保持工作状态，
                            entryIdleTick = 0;
                        }
                        else {
                            // 如果5秒内没有收到 isWork 信号，进入idle状态
                            if (entryIdleTick == 0) {
                                entryIdleTick = HAL_GetTick();
                            }
                            else {
                                if ((HAL_GetTick() - entryIdleTick) > IDLE_TIMEOUT_MS) {
                                    motorState = MotorState::IDLE_SLEEP;
                                    entryIdleTick = HAL_GetTick();
                                    LOG_I(
                                        "state: %s, duty: %d, VOPP1:%d, VOPP2:%d, VNTC:%d, "
                                        "VOCP:%d, "
                                        "WK: %d, OC: "
                                        "%d, OTD: %d, "
                                        "OTO: %d",
                                        motorStateStr[(uint8_t)motorState],
                                        _actualDuty,
                                        adcValue[ADC_IDX_OPP1],
                                        adcValue[ADC_IDX_OPP2],
                                        adcValue[ADC_IDX_NTC],
                                        adcValue[ADC_IDX_OCP],
                                        isWork,
                                        isOverCurrent,
                                        isOverTemp,
                                        isOverTempOil);
                                    LOG_I("Entry IDLE_SLEEP state");
                                }
                            }
                        }
                    }
                    else if (motorState == MotorState::IDLE_SLEEP) {
                        if ((HAL_GetTick() - entryIdleTick) >= IDLE_SLEEP_TIMEOUT_MS) {
                            motorState = MotorState::IDLE;
                            entryIdleTick = HAL_GetTick();
                            LOG_I(
                                "state: %s, duty: %d, VOPP1:%d, VOPP2:%d, VNTC:%d, VOCP:%d, WK: %d, "
                                "OC: "
                                "%d, OTD: %d, "
                                "OTO: %d",
                                motorStateStr[(uint8_t)motorState],
                                _actualDuty,
                                adcValue[ADC_IDX_OPP1],
                                adcValue[ADC_IDX_OPP2],
                                adcValue[ADC_IDX_NTC],
                                adcValue[ADC_IDX_OCP],
                                isWork,
                                isOverCurrent,
                                isOverTemp,
                                isOverTempOil);
                            LOG_I("Entry IDLE state");
                        }
                    }
                    else if (motorState == MotorState::IDLE) {
                        if (isWork) {
                            motorState = MotorState::WORK;
                            LOG_I(
                                "state: %s, duty: %d, VOPP1:%d, VOPP2:%d, VNTC:%d, VOCP:%d, WK: %d, "
                                "OC: "
                                "%d, OTD: %d, "
                                "OTO: %d",
                                motorStateStr[(uint8_t)motorState],
                                _actualDuty,
                                adcValue[ADC_IDX_OPP1],
                                adcValue[ADC_IDX_OPP2],
                                adcValue[ADC_IDX_NTC],
                                adcValue[ADC_IDX_OCP],
                                isWork,
                                isOverCurrent,
                                isOverTemp,
                                isOverTempOil);
                            LOG_I("Entry WORK state");
                        }
                    }
                    else if (motorState == MotorState::ERROR) {
                        motorState = MotorState::WORK;
                        LOG_I(
                            "state: %s, duty: %d, VOPP1:%d, VOPP2:%d, VNTC:%d, VOCP:%d, WK: %d, OC: "
                            "%d, OTD: %d, "
                            "OTO: %d",
                            motorStateStr[(uint8_t)motorState],
                            _actualDuty,
                            adcValue[ADC_IDX_OPP1],
                            adcValue[ADC_IDX_OPP2],
                            adcValue[ADC_IDX_NTC],
                            adcValue[ADC_IDX_OCP],
                            isWork,
                            isOverCurrent,
                            isOverTemp,
                            isOverTempOil);
                        LOG_I("Entry WORK state");
                    }
                }

                if (motorState == MotorState::WORK) {
                    _targetDuty = DUTY_MAX;
                }
                else if (motorState == MotorState::IDLE_SLEEP) {
                    _targetDuty = IDEL_DUTY;
                }
                else if (motorState == MotorState::IDLE) {
                    _targetDuty = IDEL_DUTY;
                }
                else {
                    _targetDuty = 0;
                }

                if (_actualDuty == _targetDuty) {
                }
                else if (_actualDuty < _targetDuty) {
                    _actualDuty += SOFT_ON_DUTY_INC_STEP;
                    if (_actualDuty > _targetDuty) {
                        _actualDuty = _targetDuty;
                    }
                }
                else if (_actualDuty > _targetDuty) {
                    _actualDuty -= SOFT_OFF_DUTY_DEC_STEP;
                    if (_actualDuty < _targetDuty) {
                        _actualDuty = _targetDuty;
                    }
                }

                setDuty(_actualDuty);
                LOG_I_INTERVAL(100,
                    "state: %s, duty: %d, VOPP1:%d, VOPP2:%d, VNTC:%d, VOCP:%d, WK: %d, OC: "
                    "%d, OTD: %d, "
                    "OTO: %d",
                    motorStateStr[(uint8_t)motorState],
                    _actualDuty,
                    adcValue[ADC_IDX_OPP1],
                    adcValue[ADC_IDX_OPP2],
                    adcValue[ADC_IDX_NTC],
                    adcValue[ADC_IDX_OCP],
                    isWork,
                    isOverCurrent,
                    isOverTemp,
                    isOverTempOil);
                HAL_Delay(1);
            }
        }

    private:
        uint16_t _actualDuty = 0;
        uint16_t _targetDuty = 0;
        MotorState motorState;
        uint32_t entryIdleTick = 0;
        uint32_t entryIdleSleepTick = 0;
        uint32_t error_tick;
};

App app;
