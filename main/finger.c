#include "finger.h"
#include "control.h"
#include "freertos/portmacro.h"
#include "beep.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "hal/uart_types.h"
#include "sdkconfig.h"
#include "util.h"

// bit0: 是否开始读指纹数据包；bit1: 是否正在验证指纹过程
static EventGroupHandle_t finger_event;

static uint8_t Head[] = {0xEF, 0x01}; // 数据包头
static uint8_t Address[] = {0xFF, 0xFF, 0xFF, 0xFF}; // 地址
static uint8_t AutoIdentify[] = {0x01, 0x00, 0x08, 0x32, 0x03, 0xFF, 0xFF, 0x00, 0x01}; // 自动验证指令码，和值校验
static uint8_t ValidTempleteNum[] = {0x01, 0x00, 0x03, 0x1D}; // 查询指纹数量指令码
static uint8_t AutoEnroll[] = {0x01, 0x00, 0x08, 0x31, 0x00, 0xFF, 0x04, 0x00, 0x55}; // 注册指纹指令码
static uint8_t CancelCmd[] = {0x01, 0x00, 0x03, 0x30, 0x00, 0x34}; //  取消命令（已加校验码）
static uint8_t Checksum[] = {0, 0}; // 储存和值校验

static void set_AutoEnroll_id(uint8_t id) {
  AutoEnroll[5] = id;
}

static void check_sum(uint8_t *data, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
  }
  Checksum[0] = (uint8_t)(sum >> 8);
  Checksum[1] = (uint8_t)sum;
}

void finger_verify(void) {
  if (xEventGroupGetBits(finger_event) & 0b10) return;
  xEventGroupSetBits(finger_event, 0b10);

  uart_flush(FINGER_UART_PORT_NUM);
  check_sum(AutoIdentify, sizeof(AutoIdentify));
  uart_write_bytes(FINGER_UART_PORT_NUM, Head, sizeof(Head));
  uart_write_bytes(FINGER_UART_PORT_NUM, Address, sizeof(Address));
  uart_write_bytes(FINGER_UART_PORT_NUM, AutoIdentify, sizeof(AutoIdentify));
  uart_write_bytes(FINGER_UART_PORT_NUM, Checksum, sizeof(Checksum));
  uart_wait_tx_done(FINGER_UART_PORT_NUM, 100 / portTICK_PERIOD_MS);

  xEventGroupSetBits(finger_event, 0b01);

}

void finger_enroll(void) {
  uart_flush(FINGER_UART_PORT_NUM);

  check_sum(ValidTempleteNum, sizeof(ValidTempleteNum));
  uart_write_bytes(FINGER_UART_PORT_NUM, Head, sizeof(Head));
  uart_write_bytes(FINGER_UART_PORT_NUM, Address, sizeof(Address));
  uart_write_bytes(FINGER_UART_PORT_NUM, ValidTempleteNum, sizeof(ValidTempleteNum));
  uart_write_bytes(FINGER_UART_PORT_NUM, Checksum, sizeof(Checksum));
  uart_wait_tx_done(FINGER_UART_PORT_NUM, 100 / portTICK_PERIOD_MS);

  uint8_t data[14];
  int len = uart_read_bytes(FINGER_UART_PORT_NUM, data, 14,
                              200 / portTICK_PERIOD_MS);
  if (len == 14 && data[9] == 0x00) {
    ESP_LOGI("FINGER_ENROLL", "New finger ID is: %x", data[11]);
    set_AutoEnroll_id(data[11]);
  } else {
    ESP_LOGE("FINGER_ENROLL", "Get finger ID failed");
    return;
  }
  
  check_sum(AutoEnroll, sizeof(AutoEnroll));
  uart_write_bytes(FINGER_UART_PORT_NUM, Head, sizeof(Head));
  uart_write_bytes(FINGER_UART_PORT_NUM, Address, sizeof(Address));
  uart_write_bytes(FINGER_UART_PORT_NUM, AutoEnroll, sizeof(AutoEnroll));
  uart_write_bytes(FINGER_UART_PORT_NUM, Checksum, sizeof(Checksum));
  
}

void finger_verify_done() {
  vTaskDelay(FINGER_GREEN_DELAY / portTICK_PERIOD_MS);

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

static void finger_read_task(void *args) {
  uint8_t data[17];
  TickType_t ticks = 0;
  while (1) {
    xEventGroupWaitBits(finger_event, 0b01, pdFALSE, pdFALSE, portMAX_DELAY);
    ticks = xTaskGetTickCount();

    ESP_LOGI("FINGER_VERIFY", "Reading response...");
    int len = uart_read_bytes(FINGER_UART_PORT_NUM, data, 17,
                              100 / portTICK_PERIOD_MS);
    if (len == 17) {
      if (data[10] == 0x01 && data[9] != 0x00) {
        // 图像采集失败
        ESP_LOGE("FINGER_VERIFY", "No valid touch");
        xEventGroupClearBits(finger_event, 0b11);
        continue;
      }
      if (data[10] == 0x05) {
        // 获取指纹比对数据成功
        if (data[9] == 0x00) {
          ESP_LOGI("FINGER_VERIFY", "Finger Verified");
          beep_ok();
          vTaskDelay(FINGER_SUCCEED_ACTION_DELAY / portTICK_PERIOD_MS);
          door_control(OPEN_AND_CLOSE);
          finger_verify_done();
        } else {
          ESP_LOGI("FINGER_VERIFY", "Finger not found");
          beep_error();
        }
        xEventGroupClearBits(finger_event, 0b11);
        continue;
      }
    }

    if ((xTaskGetTickCount() - ticks) > (FINGER_VERIFY_TIMEOUT / portTICK_PERIOD_MS)) {
      ESP_LOGE("FINGER_VERIFY", "Wait timeout");
      xEventGroupClearBits(finger_event, 0b11);
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
