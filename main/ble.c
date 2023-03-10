#include "ble.h"
#include "control.h"
#include "finger.h"
#include "ibeacon.h"
#include "util.h"

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

static const char *tag = "NimBLE";

static void ble_control(uint8_t *data, uint8_t data_len) {

  if (data_len < 18)
    return;

  // 开门
  static uint8_t open[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                           0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                           0xCB, 0xCF, 0x65, 0xDA, 0x51, 0x3B};
  if (compare_byte(data, open, sizeof(open)) == 0) {
    ESP_LOGI("BLE", "Key match: open");
    door_control(OPEN);
    return;
  }

  // 停
  static uint8_t stop[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                           0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                           0xCB, 0xCF, 0x65, 0xBA, 0x57, 0x58};
  if (compare_byte(data, stop, sizeof(stop)) == 0) {
    ESP_LOGI("BLE", "Key match: stop");
    door_control(STOP);
    return;
  }

  // 关门
  static uint8_t close[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                            0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                            0xCB, 0xCF, 0x65, 0x7A, 0x5B, 0x9E};
  if (compare_byte(data, close, sizeof(close)) == 0) {
    ESP_LOGI("BLE", "Key match: close");
    door_control(CLOSE);
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
    door_control(OPEN_AND_CLOSE);
    return;
  }

  // 参数设置
  static uint8_t set_ibeacon_config[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43,
                                         0x4F, 0x9E, 0x0F, 0x87, 0x91,
                                         0x23, 0x6F, 0xCB, 0xCF, 0x6A};
  if (compare_byte(data, set_ibeacon_config, sizeof(set_ibeacon_config)) == 0) {
    ESP_LOGI("BLE", "Key match: set_ibeacon_config");
    uint32_t config = (data[15] << 16) + (data[16] << 8) + data[17];
    save_ibeacon_config(config);
    return;
  }
}

static int blecent_gap_event(struct ble_gap_event *event, void *arg) {
  struct ble_hs_adv_fields fields;
  int rc;

  if (event->type == BLE_GAP_EVENT_DISC) {

    if (check_ibeacon(event->disc.addr.val, event->disc.rssi)) {
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
  nimble_port_init();
  /* Configure the host. */
  ble_hs_cfg.reset_cb = blecent_on_reset;
  ble_hs_cfg.sync_cb = blecent_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  /* Set the default device name. */
  rc = ble_svc_gap_device_name_set("ESP32");
  assert(rc == 0);

  nimble_port_freertos_init(blecent_host_task);
}
