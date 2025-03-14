#include "config.h"
#include "main.h"
#include "stm32g0xx_hal.h"

#include "logger.hpp"
#include "app-framework.hpp"
LOGGER("app")

using namespace wibot;

class App : public AppFramework {
   public:
    App() : AppFramework(APP_NAME, APP_VERSION) {
    }
    void onRun() override {
    }

   private:
    uint32_t error_tick;
};

App app;
