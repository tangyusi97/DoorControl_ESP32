#include "finger.h"
#include "beep.h"
#include "control.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "hal/uart_types.h"

// bit0: 是否开始读指纹数据包；bit1: 是否正在验证指纹过程；bit2: 是否在注册指纹
static EventGroupHandle_t finger_event;

static uint8_t Head[] = {0xEF, 0x01};                // 数据包头
static uint8_t Address[] = {0xFF, 0xFF, 0xFF, 0xFF}; // 地址
static uint8_t AutoIdentify[] = {0x01, 0x00, 0x08, 0x32, 0x03,
                                 0xFF, 0xFF, 0x00, 0x01}; // 自动验证指纹
static uint8_t ValidTempleteNum[] = {0x01, 0x00, 0x03, 0x1D}; // 查询指纹数量
static uint8_t AutoEnroll[] = {0x01, 0x00, 0x08, 0x31, 0x00,
                               0xFF, 0x04, 0x00, 0x51}; // 注册指纹
static uint8_t CancelCmd[] = {0x01, 0x00, 0x03,
                              0x30, 0x00, 0x34}; //  取消命令（已加校验码）
static uint8_t Checksum[] = {0, 0};              // 储存和值校验

static void set_AutoEnroll_id(uint8_t id) { AutoEnroll[5] = id; }

static void check_sum(uint8_t *data, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }
  Checksum[0] = (uint8_t)(sum >> 8);
  Checksum[1] = (uint8_t)sum;
}

void finger_verify(void) {
  if (xEventGroupGetBits(finger_event) & 0b110)
    return;
  xEventGroupSetBits(finger_event, 0b010);

  uart_flush(FINGER_UART_PORT_NUM);
  check_sum(AutoIdentify, sizeof(AutoIdentify));
  uart_write_bytes(FINGER_UART_PORT_NUM, Head, sizeof(Head));
  uart_write_bytes(FINGER_UART_PORT_NUM, Address, sizeof(Address));
  uart_write_bytes(FINGER_UART_PORT_NUM, AutoIdentify, sizeof(AutoIdentify));
  uart_write_bytes(FINGER_UART_PORT_NUM, Checksum, sizeof(Checksum));
  uart_wait_tx_done(FINGER_UART_PORT_NUM, 100 / portTICK_PERIOD_MS);

  xEventGroupSetBits(finger_event, 0b001);
}

static void finger_verify_done() {
  check_sum(AutoIdentify, sizeof(AutoIdentify));
  uart_write_bytes(FINGER_UART_PORT_NUM, Head, sizeof(Head));
  uart_write_bytes(FINGER_UART_PORT_NUM, Address, sizeof(Address));
  uart_write_bytes(FINGER_UART_PORT_NUM, AutoIdentify, sizeof(AutoIdentify));
  uart_write_bytes(FINGER_UART_PORT_NUM, Checksum, sizeof(Checksum));

  vTaskDelay(100 / portTICK_PERIOD_MS);

  uart_write_bytes(FINGER_UART_PORT_NUM, Head, sizeof(Head));
  uart_write_bytes(FINGER_UART_PORT_NUM, Address, sizeof(Address));
  uart_write_bytes(FINGER_UART_PORT_NUM, CancelCmd, sizeof(CancelCmd));
}

void finger_enroll(void) {
  if (xEventGroupGetBits(finger_event) & 0b110)
    return;
  xEventGroupSetBits(finger_event, 0b100);
  uart_flush(FINGER_UART_PORT_NUM);

  check_sum(ValidTempleteNum, sizeof(ValidTempleteNum));
  uart_write_bytes(FINGER_UART_PORT_NUM, Head, sizeof(Head));
  uart_write_bytes(FINGER_UART_PORT_NUM, Address, sizeof(Address));
  uart_write_bytes(FINGER_UART_PORT_NUM, ValidTempleteNum,
                   sizeof(ValidTempleteNum));
  uart_write_bytes(FINGER_UART_PORT_NUM, Checksum, sizeof(Checksum));
  uart_wait_tx_done(FINGER_UART_PORT_NUM, 100 / portTICK_PERIOD_MS);

  uint8_t data[14];
  int len =
      uart_read_bytes(FINGER_UART_PORT_NUM, data, 14, 200 / portTICK_PERIOD_MS);
  if (len == 14 && data[9] == 0x00) {
    ESP_LOGI("FINGER", "New finger ID is: %x", data[11]);
    set_AutoEnroll_id(data[11]);
  } else {
    ESP_LOGE("FINGER", "Get finger ID failed");
    xEventGroupClearBits(finger_event, 0b100);
    return;
  }

  check_sum(AutoEnroll, sizeof(AutoEnroll));
  uart_write_bytes(FINGER_UART_PORT_NUM, Head, sizeof(Head));
  uart_write_bytes(FINGER_UART_PORT_NUM, Address, sizeof(Address));
  uart_write_bytes(FINGER_UART_PORT_NUM, AutoEnroll, sizeof(AutoEnroll));
  uart_write_bytes(FINGER_UART_PORT_NUM, Checksum, sizeof(Checksum));

  xEventGroupSetBits(finger_event, 0b001);
}

static void finger_read_task(void *args) {
  uint8_t data[17];
  uint8_t read_type = 0b000;
  uint8_t read_len = 0;
  while (1) {
    read_type = xEventGroupWaitBits(finger_event, 0b110, pdFALSE, pdFALSE,
                                    portMAX_DELAY);
    if (read_type & 0b010) {
      read_len = 17; // 指纹验证
    }
    if (read_type & 0b100) {
      read_len = 14; // 指纹注册
    }
    xEventGroupWaitBits(finger_event, 0b001, pdFALSE, pdFALSE, portMAX_DELAY);

    ESP_LOGI("FINGER", "Reading response...");
    int len = uart_read_bytes(FINGER_UART_PORT_NUM, data, read_len,
                              FINGER_TIMEOUT / portTICK_PERIOD_MS);
    if (len == 17) {
      if (data[9] == 0x24 && data[10] == 0x00) {
        // 指纹库为空
        ESP_LOGE("FINGER", "Finger library is empty");
        goto error;
      }
      if (data[9] == 0x26 && data[10] == 0x00) {
        // 指纹采集超时
        ESP_LOGE("FINGER", "Finger read timeout");
        goto error;
      }
      if (data[9] == 0x09 && data[10] == 0x05) {
        // 指纹比对失败
        ESP_LOGE("FINGER", "Finger not found");
        goto error;
      }
      if (data[9] == 0x00 && data[10] == 0x05) {
        // 获取指纹比对数据成功
        ESP_LOGI("FINGER", "Finger Verified");
        beep_ok();
        vTaskDelay(FINGER_SUCCEED_ACTION_DELAY / portTICK_PERIOD_MS);
        door_control(OPEN_AND_CLOSE);
        finger_verify_done();
        xEventGroupClearBits(finger_event, 0b111);
        continue;
      }
    }

    if (len == 14) {
      if (data[9] == 0x0b && data[10] == 0x00 && data[11] == 0x00) {
        // 指纹ID无效
        ESP_LOGE("FINGER", "Finger ID is invalid");
        goto error;
      }
      if (data[9] == 0x25 && data[10] == 0x00 && data[11] == 0x00) {
        // 录入次数配置错误
        ESP_LOGE("FINGER", "Record number set error");
        goto error;
      }
      if (data[9] == 0x1f && data[10] == 0x00 && data[11] == 0x00) {
        // 指纹库已满
        ESP_LOGE("FINGER", "Finger library is full");
        goto error;
      }
      if (data[9] == 0x22 && data[10] == 0x00 && data[11] == 0x00) {
        // 指纹ID冲突
        ESP_LOGE("FINGER", "Finger ID is conflict");
        goto error;
      }
      if (data[9] == 0x0A && data[10] == 0x04 && data[11] == 0xF0) {
        // 指纹特征生产失败
        ESP_LOGE("FINGER", "Finger build failed");
        goto error;
      }
      if (data[9] == 0x27 && data[10] == 0x05 && data[11] == 0xF1) {
        // 已录入该指纹
        ESP_LOGI("FINGER", "Finger has enrolled");
        goto error;
      }
      if (data[9] == 0x01 && data[10] == 0x06 && data[11] == 0xF2) {
        // 指纹存储失败
        ESP_LOGE("FINGER", "Finger record failed");
        goto error;
      }
      if (data[9] == 0x00 && data[10] == 0x06 && data[11] == 0xF2) {
        // 指纹录入成功
        ESP_LOGI("FINGER", "Finger enroll success");
        xEventGroupClearBits(finger_event, 0b111);
        continue;
      }
    }

    if (len <= 0) {
      ESP_LOGE("FINGER", "Wait timeout");

    error:
      xEventGroupClearBits(finger_event, 0b111);
      beep_error();
    }
  }
}

static void detect_finger_task(void *args) {
  gpio_reset_pin(FINGER_TOUCH_GPIO);
  gpio_set_direction(FINGER_TOUCH_GPIO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(FINGER_TOUCH_GPIO, GPIO_PULLDOWN_ONLY);

  while (1) {
    if (gpio_get_level(FINGER_TOUCH_GPIO)) {
      finger_verify();
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void finger_init(void) {
  uart_config_t uart_config = {
      .baud_rate = FINGER_UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_2,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  int intr_alloc_flags = 0;

  ESP_ERROR_CHECK(uart_driver_install(FINGER_UART_PORT_NUM,
                                      FINGER_UART_BUF_SIZE * 2, 0, 0, NULL,
                                      intr_alloc_flags));
  ESP_ERROR_CHECK(uart_param_config(FINGER_UART_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(FINGER_UART_PORT_NUM, FINGER_TXD, FINGER_RXD,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  finger_event = xEventGroupCreate();
  xTaskCreate(finger_read_task, "finger_read_task", 2048, NULL, 10, NULL);
  xTaskCreate(detect_finger_task, "detect_finger_task", 2048, NULL, 12, NULL);
}
