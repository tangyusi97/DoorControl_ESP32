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
#include "nvs_flash.h"

static void restart_callback(TimerHandle_t xTimer) { esp_restart(); }

/* Initialize NVS — it is used to store data */
static void nvs_init(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void app_main(void) {
  beep_init();
  finger_init();
  control_init();
  
  // 需要数据持久化
  nvs_init();
  ibeacon_init();
  ble_init();

  TimerHandle_t restart_timer =
      xTimerCreate("restart", 7 * 24 * 3600 * 1000 / portTICK_PERIOD_MS, pdTRUE,
                   NULL, restart_callback);
  xTimerStart(restart_timer, 0);
}
