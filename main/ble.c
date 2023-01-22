#include "ble.h"
#include "finger.h"
#include "gpio.h"
#include "util.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

static const char *tag = "NimBLE";

static uint8_t ibeacon1_index = 0;
static uint8_t ibeacon2_index = 1;
static uint8_t ibeacon_rssi[2][IBEACON_AVG_WINDOW];
static uint8_t ibeacon_tick[2] = {0, 0};
static uint8_t is_ibeacon_valid[2] = {0, 0};

static QueueHandle_t ibeacon_rssi_queue[2];

static void ble_control(uint8_t *data, uint8_t data_len) {

  if (data_len < 17)
    return;

  // 开门
  static uint8_t open[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                           0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                           0xCB, 0xCF, 0x65, 0xDA, 0x51, 0x3B};
  if (compare_byte(data, open, sizeof(open)) == 0) {
    ESP_LOGI("BLE", "Key match: open");
    door_open();
    return;
  }

  // 停
  static uint8_t stop[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                           0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                           0xCB, 0xCF, 0x65, 0xBA, 0x57, 0x58};
  if (compare_byte(data, stop, sizeof(stop)) == 0) {
    ESP_LOGI("BLE", "Key match: stop");
    door_stop();
    return;
  }

  // 关门
  static uint8_t close[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                            0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                            0xCB, 0xCF, 0x65, 0x7A, 0x5B, 0x9E};
  if (compare_byte(data, close, sizeof(close)) == 0) {
    ESP_LOGI("BLE", "Key match: close");
    door_close();
    return;
  }

  // 注册
  static uint8_t enroll[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                             0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                             0xCB, 0xCF, 0x65, 0xC9, 0x54, 0x8D};
  if (compare_byte(data, enroll, sizeof(enroll)) == 0) {
    ESP_LOGI("BLE", "Key match: enroll");
    finger_enroll();
    return;
  }

  // 开一下
  static uint8_t open_and_close[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                                     0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                                     0xCB, 0xCF, 0x65, 0x87, 0x5E, 0xA4};
  if (compare_byte(data, open_and_close, sizeof(open_and_close)) == 0) {
    ESP_LOGI("BLE", "Key match: open_and_close");
    door_open_and_close();
    return;
  }

  // TODO 遥控器按键学习
}

// 检测iBeacon设备
static uint8_t check_ibeacon(struct ble_gap_event *event, uint8_t ibeacon_num) {
  static uint8_t ibeacon_addr[2][6] = {
      {0x39, 0x03, 0x12, 0x22, 0x00, 0x51},  // ibeacon1
      {0x3A, 0x03, 0x12, 0x22, 0x00, 0x51}}; // ibeacon2
  if (compare_byte(event->disc.addr.val, ibeacon_addr[ibeacon_num], 6) == 0) {
    ibeacon_rssi[ibeacon_num][ibeacon_tick[ibeacon_num]] = -event->disc.rssi;
    ibeacon_tick[ibeacon_num]++;
    if (ibeacon_tick[ibeacon_num] == IBEACON_AVG_WINDOW) {
      ibeacon_tick[ibeacon_num] = 0;
    }
    uint16_t ibeacon_rssi_avg = 0;
    for (uint8_t i = 0; i < IBEACON_AVG_WINDOW; i++) {
      ibeacon_rssi_avg += ibeacon_rssi[ibeacon_num][i];
    }
    ibeacon_rssi_avg /= IBEACON_AVG_WINDOW;
    xQueueSend(ibeacon_rssi_queue[ibeacon_num], &ibeacon_rssi_avg, 0);
    return 0;
  }
  return 1;
}

// iBeacon设备距离判定
static void check_ibeacon_distance_task(void *args) {
  uint16_t ibeacon_rssi_avg;
  uint8_t *ibeacon_num = (uint8_t *)args;
  while (1) {
    BaseType_t received =
        xQueueReceive(ibeacon_rssi_queue[*ibeacon_num], &ibeacon_rssi_avg,
                      IBEACON_LEAVE_TIMEOUT / portTICK_PERIOD_MS);
    if (received == pdFALSE) {
      is_ibeacon_valid[*ibeacon_num] = 0;
      continue;
    }
    if (is_ibeacon_valid[*ibeacon_num] == 0 &&
        ibeacon_rssi_avg >= IBEACON_ENTER_RSSI) {
      continue;
    }
    if (ibeacon_rssi_avg <= IBEACON_LEAVE_RSSI) {
      is_ibeacon_valid[*ibeacon_num] = 1;
      continue;
    }
    is_ibeacon_valid[*ibeacon_num] = 0;
  }
}

// 自动开门控制
static void door_control_task(void *args) {
  uint8_t last_state[2];
  last_state[0] = is_ibeacon_valid[0];
  last_state[1] = is_ibeacon_valid[1];
  while (1) {
    int8_t flag[2];
    flag[0] = is_ibeacon_valid[0] - last_state[0];
    flag[1] = is_ibeacon_valid[1] - last_state[1];
    last_state[0] = is_ibeacon_valid[0];
    last_state[1] = is_ibeacon_valid[1];
    if (((flag[0] > 0) && (is_ibeacon_valid[1] == 0)) ||
        ((flag[1] > 0) && (is_ibeacon_valid[0] == 0))) {
      ESP_LOGI("BEACON", "Enter");
      door_stop();
      door_open();
    } else if (((flag[0] < 0) && (is_ibeacon_valid[1] == 0)) ||
               ((flag[1] < 0) && (is_ibeacon_valid[0] == 0))) {
      ESP_LOGI("BEACON", "Leave");
      door_stop();
      door_close();
    } else {
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}

static int blecent_gap_event(struct ble_gap_event *event, void *arg) {
  struct ble_hs_adv_fields fields;
  int rc;

  if (event->type == BLE_GAP_EVENT_DISC) {

    if ((check_ibeacon(event, 0) == 0) || (check_ibeacon(event, 1) == 0)) {
      return 0;
    };

    rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                 event->disc.length_data);
    if (rc != 0) {
      return 0;
    }

    uint8_t *data = (uint8_t *)fields.mfg_data;
    uint8_t data_len = fields.mfg_data_len;
    ble_control(data, data_len);
  }

  return 0;
}

/**
 * Initiates the GAP general discovery procedure.
 */
static void blecent_scan(void) {
  uint8_t own_addr_type;
  struct ble_gap_disc_params disc_params;
  int rc;

  /* Figure out address to use while advertising (no privacy for now) */
  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    ESP_LOGE(tag, "error determining address type; rc=%d\n", rc);
    return;
  }

  /* Tell the controller to filter duplicates; we don't want to process
   * repeated advertisements from the same device.
   */
  disc_params.filter_duplicates = 0;

  /**
   * Perform a passive scan.  I.e., don't send follow-up scan requests to
   * each advertiser.
   */
  disc_params.passive = 1;

  /* Use defaults for the rest of the parameters. */
  disc_params.itvl = 0;
  disc_params.window = 0;
  disc_params.filter_policy = 0;
  disc_params.limited = 0;

  rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                    blecent_gap_event, NULL);
  if (rc != 0) {
    ESP_LOGE(tag, "Error initiating GAP discovery procedure; rc=%d\n", rc);
  }
}

static void blecent_on_reset(int reason) {
  MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void blecent_on_sync(void) {
  int rc;

  /* Make sure we have proper identity address set (public preferred) */
  rc = ble_hs_util_ensure_addr(0);
  assert(rc == 0);

  /* Begin scanning for a peripheral to connect to. */
  blecent_scan();
}

static void blecent_host_task(void *param) {
  ESP_LOGI(tag, "BLE Host Task Started");
  /* This function will return only when nimble_port_stop() is executed */
  nimble_port_run();

  nimble_port_freertos_deinit();
}

void ble_init(void) {
  int rc;
  /* Initialize NVS — it is used to store PHY calibration data */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  nimble_port_init();
  /* Configure the host. */
  ble_hs_cfg.reset_cb = blecent_on_reset;
  ble_hs_cfg.sync_cb = blecent_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  /* Set the default device name. */
  rc = ble_svc_gap_device_name_set("ESP32");
  assert(rc == 0);

  nimble_port_freertos_init(blecent_host_task);

  ibeacon_rssi_queue[0] = xQueueCreate(2, sizeof(uint16_t));
  ibeacon_rssi_queue[1] = xQueueCreate(2, sizeof(uint16_t));
  for (uint8_t i = 0; i < IBEACON_AVG_WINDOW; i++) {
    ibeacon_rssi[0][i] = 100; // 防止设备一开机判断为iBeacon进入
    ibeacon_rssi[1][i] = 100; // 防止设备一开机判断为iBeacon进入
  }
  xTaskCreate(check_ibeacon_distance_task, "check_ibeacon_distance_task1", 2048,
              &ibeacon1_index, 12, NULL);
  xTaskCreate(check_ibeacon_distance_task, "check_ibeacon_distance_task2", 2048,
              &ibeacon2_index, 12, NULL);
  xTaskCreate(door_control_task, "door_control_task", 2048, NULL, 12, NULL);
}
