#include "ibeacon.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "util.h"

#include "control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static TaskHandle_t door_auto_control_handle;
static TaskHandle_t check_ibeacon_distance_handle[IBEACONS_NUM];
static uint8_t ibeacon_rssi[IBEACONS_NUM][IBEACON_AVG_WINDOW];
static uint8_t ibeacon_tick[IBEACONS_NUM];
static uint8_t is_ibeacon_valid[IBEACONS_NUM];

// iBeacon的MAC地址，顺序是反的
static uint8_t ibeacon_addr[IBEACONS_NUM][6] = {
    {0x39, 0x03, 0x12, 0x22, 0x00, 0x51}, // ibeacon1
    {0x3A, 0x03, 0x12, 0x22, 0x00, 0x51}, // ibeacon2
};

// 检测iBeacon设备
uint8_t check_ibeacon(uint8_t *addr, uint8_t rssi) {
  uint8_t available = 0;
  for (uint8_t ibeacon_i = 0; ibeacon_i < IBEACONS_NUM; ibeacon_i++) {
    if (compare_byte(addr, ibeacon_addr[ibeacon_i], 6) == 0) {
      ibeacon_rssi[ibeacon_i][ibeacon_tick[ibeacon_i]] = -rssi;
      ibeacon_tick[ibeacon_i]++;
      if (ibeacon_tick[ibeacon_i] == IBEACON_AVG_WINDOW) {
        ibeacon_tick[ibeacon_i] = 0;
      }
      uint16_t ibeacon_rssi_avg = 0;
      for (uint8_t i = 0; i < IBEACON_AVG_WINDOW; i++) {
        ibeacon_rssi_avg += ibeacon_rssi[ibeacon_i][i];
      }
      ibeacon_rssi_avg /= IBEACON_AVG_WINDOW;
      xTaskNotify(check_ibeacon_distance_handle[ibeacon_i], ibeacon_rssi_avg,
                  eSetValueWithOverwrite);
      available = 1;
    }
  }
  return available;
}

// iBeacon设备距离判定
static void check_ibeacon_distance_task(void *args) {
  uint16_t ibeacon_rssi_avg;
  uint8_t *arg = (uint8_t *)args;
  uint8_t ibeacon_i = *arg;
  while (1) {
    ibeacon_rssi_avg =
        ulTaskNotifyTake(pdTRUE, IBEACON_LEAVE_TIMEOUT / portTICK_PERIOD_MS);
    if (ibeacon_rssi_avg == 0) {
      is_ibeacon_valid[ibeacon_i] = 0;
      goto end;
    }
    if (is_ibeacon_valid[ibeacon_i] == 0 &&
        ibeacon_rssi_avg >= IBEACON_ENTER_RSSI) {
      goto end;
    }
    if (ibeacon_rssi_avg <= IBEACON_LEAVE_RSSI) {
      is_ibeacon_valid[ibeacon_i] = 1;
      goto end;
    }
    is_ibeacon_valid[ibeacon_i] = 0;
  end:
    xTaskNotifyGive(door_auto_control_handle);
  }
}

// 自动开门控制
static void door_auto_control_task(void *args) {
  uint8_t last_state[IBEACONS_NUM];
  for (uint8_t i = 0; i < IBEACONS_NUM; i++) {
    last_state[i] = is_ibeacon_valid[i];
  }
  while (1) {
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    int8_t flag[IBEACONS_NUM];
    uint8_t ibeacon_valid_num = 0;
    uint8_t open_active = 0;
    uint8_t close_active = 0;
    for (uint8_t i = 0; i < IBEACONS_NUM; i++) {
      flag[i] = is_ibeacon_valid[i] - last_state[i];
      last_state[i] = is_ibeacon_valid[i];
      ibeacon_valid_num += is_ibeacon_valid[i];
      if (flag[i] < 0) {
        close_active = 1;
      }
      if (flag[i] > 0) {
        open_active = 1;
      }
    }
    if (ibeacon_valid_num == 1 && open_active == 1) {
      ESP_LOGI("BEACON", "Enter");
      door_control(STOP);
      door_control(OPEN);
      continue;
    }
    if (ibeacon_valid_num == 0 && close_active == 1) {
      ESP_LOGI("BEACON", "Leave");
      door_control(STOP);
      door_control(CLOSE);
      continue;
    }
  }
}

void ibeacon_init(void) {
  for (uint8_t ibeacon_i = 0; ibeacon_i < IBEACONS_NUM; ibeacon_i++) {
    xTaskCreate(check_ibeacon_distance_task, "check_ibeacon_distance_task",
                2048, &ibeacon_i, 12,
                &check_ibeacon_distance_handle[ibeacon_i]);
    for (uint8_t i = 0; i < IBEACON_AVG_WINDOW; i++) {
      ibeacon_rssi[ibeacon_i][i] = 100; // 防止设备一开机判断为iBeacon进入
    }
    ibeacon_tick[ibeacon_i] = 0;
    is_ibeacon_valid[ibeacon_i] = 0;
  }

  xTaskCreate(door_auto_control_task, "door_auto_control_task", 2048, NULL, 13,
              &door_auto_control_handle);
}