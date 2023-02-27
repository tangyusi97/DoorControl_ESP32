#include "ibeacon.h"
#include "beep.h"
#include "esp_err.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "nvs.h"
#include "util.h"

#include "control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static TaskHandle_t door_auto_control_handle;
static TaskHandle_t check_ibeacon_distance_handle[IBEACONS_NUM];
static uint8_t ibeacon_num[IBEACONS_NUM];
static uint8_t ibeacon_rssi[IBEACONS_NUM][IBEACON_AVG_WINDOW];
static uint8_t ibeacon_tick[IBEACONS_NUM];
static uint8_t is_ibeacon_valid[IBEACONS_NUM];

static uint8_t ibeacon_enter_rssi = IBEACON_ENTER_RSSI_DEFAULT;
static uint8_t ibeacon_enter_debounce = IBEACON_ENTER_DEBOUNCE_DEFAULT;
static uint8_t ibeacon_leave_rssi = IBEACON_LEAVE_RSSI_DEFAULT;

// iBeacon的MAC地址，顺序是反的
static uint8_t ibeacon_addr[IBEACONS_NUM][6] = {
    {0x06, 0x01, 0x23, 0x05, 0x22, 0x20}, // ibeacon1
    {0x0A, 0x01, 0x23, 0x05, 0x22, 0x20}, // ibeacon2
    // {0x39, 0x03, 0x12, 0x22, 0x00, 0x51}, // ibeacon1
    // {0x3A, 0x03, 0x12, 0x22, 0x00, 0x51}, // ibeacon2
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
      ESP_LOGD("IBEACON", "%d", ibeacon_rssi_avg);
      xTaskNotify(check_ibeacon_distance_handle[ibeacon_i], ibeacon_rssi_avg,
                  eSetValueWithOverwrite);
      available = 1;
      break;
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
    uint8_t last_state = is_ibeacon_valid[ibeacon_i];
    if (ibeacon_rssi_avg == 0) {
      // 信号捕获超时
      is_ibeacon_valid[ibeacon_i] = 0;
      goto end;
    }
    if (is_ibeacon_valid[ibeacon_i] < ibeacon_enter_debounce &&
        ibeacon_rssi_avg > ibeacon_enter_rssi) {
      // iBeacon未进入有效范围，清零防抖计数
      is_ibeacon_valid[ibeacon_i] = 0;
      continue;
    }
    // iBeacon已进入有效范围，判断是否离开
    if (ibeacon_rssi_avg < ibeacon_leave_rssi) {
      if (is_ibeacon_valid[ibeacon_i] < ibeacon_enter_debounce) {
        is_ibeacon_valid[ibeacon_i]++;
      }
    } else {
      is_ibeacon_valid[ibeacon_i] = 0;
    }

  end:
    if (last_state != is_ibeacon_valid[ibeacon_i]) {
      if (is_ibeacon_valid[ibeacon_i] == ibeacon_enter_debounce) {
        // valid超过防抖值，开门
        xTaskNotify(door_auto_control_handle, 2, eSetValueWithOverwrite);
      };
      if (is_ibeacon_valid[ibeacon_i] == 0) {
        // valid变为0，关门
        xTaskNotify(door_auto_control_handle, 1, eSetValueWithOverwrite);
      };
    }
  }
}

// 自动开门控制
static void door_auto_control_task(void *args) {
  while (1) {
    // 1:关门；2：开门；0：无效
    uint8_t flag = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint8_t ibeacon_valid_num = 0;
    for (uint8_t i = 0; i < IBEACONS_NUM; i++) {
      if (is_ibeacon_valid[i] == ibeacon_enter_debounce) {
        ibeacon_valid_num++;
      }
    }
    if (ibeacon_valid_num == 1 && flag == 2) {
      ESP_LOGI("BEACON", "Enter");
      door_control(STOP);
      door_control(OPEN);
      continue;
    }
    if (ibeacon_valid_num == 0 && flag == 1) {
      ESP_LOGI("BEACON", "Leave");
      door_control(STOP);
      door_control(CLOSE);
      continue;
    }
  }
}

static void load_ibeacon_config(void) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("ibeacon", NVS_READONLY, &my_handle);
  if (err == ESP_OK) {
    uint32_t ibeacon_config = 0;
    err = nvs_get_u32(my_handle, "ibeacon_config", &ibeacon_config);
    if (err == ESP_OK) {
      ibeacon_enter_rssi = ibeacon_config >> 16;
      ibeacon_enter_debounce = ibeacon_config >> 8;
      ibeacon_leave_rssi = ibeacon_config;
      ESP_LOGI("IBEACON", "Config Load: enter:%d, debounce:%d, leave:%d",
               ibeacon_enter_rssi, ibeacon_enter_debounce, ibeacon_leave_rssi);
    }
    nvs_close(my_handle);
  }
}

void save_ibeacon_config(uint32_t config) {
  uint8_t _ibeacon_enter_rssi = config >> 16;
  uint8_t _ibeacon_enter_debounce = config >> 8;
  uint8_t _ibeacon_leave_rssi = config;
  if (ibeacon_enter_rssi == _ibeacon_enter_rssi &&
      ibeacon_enter_debounce == _ibeacon_enter_debounce &&
      ibeacon_leave_rssi == _ibeacon_leave_rssi) {
    return;
  }
  ibeacon_enter_rssi = _ibeacon_enter_rssi;
  ibeacon_enter_debounce = _ibeacon_enter_debounce;
  ibeacon_leave_rssi = _ibeacon_leave_rssi;
  ESP_LOGI("IBEACON", "Config Save: enter:%d, debounce:%d, leave:%d",
           ibeacon_enter_rssi, ibeacon_enter_debounce, ibeacon_leave_rssi);

  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("ibeacon", NVS_READWRITE, &my_handle);
  if (err == ESP_OK) {
    uint32_t ibeacon_config = config;
    nvs_set_u32(my_handle, "ibeacon_config", ibeacon_config);
    nvs_commit(my_handle);
    nvs_close(my_handle);
  }
}

void ibeacon_init(void) {
  for (uint8_t ibeacon_i = 0; ibeacon_i < IBEACONS_NUM; ibeacon_i++) {
    ibeacon_num[ibeacon_i] = ibeacon_i;
    xTaskCreate(check_ibeacon_distance_task, "check_ibeacon_distance_task",
                2048, &ibeacon_num[ibeacon_i], 12,
                &check_ibeacon_distance_handle[ibeacon_i]);
    for (uint8_t i = 0; i < IBEACON_AVG_WINDOW; i++) {
      ibeacon_rssi[ibeacon_i][i] = 100; // 防止设备一开机判断为iBeacon进入
    }
    ibeacon_tick[ibeacon_i] = 0;
    is_ibeacon_valid[ibeacon_i] = 0;
  }
  load_ibeacon_config();

  xTaskCreate(door_auto_control_task, "door_auto_control_task", 2048, NULL, 13,
              &door_auto_control_handle);
}