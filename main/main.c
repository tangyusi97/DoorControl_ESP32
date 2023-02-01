#include "beep.h"
#include "ble.h"
#include "control.h"
#include "finger.h"

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void restart_task(void *args) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  xTaskDelayUntil(&xLastWakeTime, 7* 24 * 3600 * 1000 / portTICK_PERIOD_MS);
  esp_restart();
  while (1);
}

void app_main(void) {
  beep_init();
  control_init();
  finger_init();
  ble_init();

  xTaskCreate(restart_task, "restart_task", 1024, NULL, 20, NULL);
}
