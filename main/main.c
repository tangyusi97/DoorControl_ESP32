#include "beep.h"
#include "ble.h"
#include "control.h"
#include "finger.h"
#include "ibeacon.h"

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "freertos/timers.h"


static void restart_callback(TimerHandle_t xTimer) { esp_restart(); }

void app_main(void) {
  beep_init();
  finger_init();
  control_init();
  ibeacon_init();
  ble_init();

  TimerHandle_t restart_timer =
      xTimerCreate("restart", 7 * 24 * 3600 * 1000 / portTICK_PERIOD_MS, pdTRUE,
                   NULL, restart_callback);
  xTimerStart(restart_timer, 0);
}
