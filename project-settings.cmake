# TODO: 项目名称
set(PROJECT_NAME "AQ-MDS24")

set(WIBOTLIB_PATH "libs/wibotlib4arm")

# TODO: 目标MCU
set(TARGET_MCU "STM32G031G8U3")

option(USE_LL_LIB "Enable LL library" ON)
option(USE_HAL_LIB "Enable HAL library" ON)
option(USE_CMSIS_DSP_LIB "Enable CMSIS DSP library" OFF)
option(USE_SYSTEM_VIEW "Enable Segger SystemView library" OFF)

set(SYSCALL "RTT")

option(LOG_COLOR "Enable Colorful log line" OFF)
set(LOG_LEVEL "DEBUG")

option(ENABLE_DEMO "Enable demo" OFF)

# set(OS "threadx")
set(OS "nortos")
# set(OS "freertos")
