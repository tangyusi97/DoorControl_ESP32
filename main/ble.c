#include "ble.h"
#include "finger.h"
#include "gpio.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "util.h"

static const char *tag = "NimBLE";

static void ble_control(uint8_t *data, uint8_t data_len) {

  if (data_len < 1)
    return;

  // 开门
  static uint8_t open[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                           0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                           0xCB, 0xCF, 0x65, 0xDA, 0x51, 0x3B};
  if (compare_byte(data, open, sizeof(open)) == 0) {
    ESP_LOGI("CONTROL", "open");
    door_open();
    return;
  }

  // 停
  static uint8_t stop[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                           0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                           0xCB, 0xCF, 0x65, 0xBA, 0x57, 0x58};
  if (compare_byte(data, stop, sizeof(stop)) == 0) {
    ESP_LOGI("CONTROL", "stop");
    door_stop();
    return;
  }

  // 关门
  static uint8_t close[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                            0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                            0xCB, 0xCF, 0x65, 0x7A, 0x5B, 0x9E};
  if (compare_byte(data, close, sizeof(close)) == 0) {
    ESP_LOGI("CONTROL", "close");
    door_close();
    return;
  }

  // 注册
  static uint8_t enroll[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                             0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                             0xCB, 0xCF, 0x65, 0xC9, 0x54, 0x8D};
  if (compare_byte(data, enroll, sizeof(enroll)) == 0) {
    ESP_LOGI("CONTROL", "enroll");
    finger_enroll();
    return;
  }

  // 开一下
  static uint8_t open_and_close[] = {0xF0, 0xFF, 0x6D, 0xB6, 0x43, 0x4F,
                                     0x9E, 0x0F, 0x87, 0x91, 0x23, 0x6F,
                                     0xCB, 0xCF, 0x65, 0x87, 0x5E, 0xA4};
  if (compare_byte(data, open_and_close, sizeof(open_and_close)) == 0) {
    ESP_LOGI("CONTROL", "open_and_close");
    door_open_and_close();
    return;
  }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  blecent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int blecent_gap_event(struct ble_gap_event *event, void *arg) {
  struct ble_hs_adv_fields fields;
  int rc;

  if (event->type == BLE_GAP_EVENT_DISC) {
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
  disc_params.filter_duplicates = 1;

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
}
