#include "rf.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdint.h>

static gptimer_handle_t gptimer;
static QueueHandle_t rf_send_queue;
static QueueHandle_t rf_data_queue;

static uint32_t rf_data[3] = {
    0xE7AAA1, // 开
    0xE7AAA8, // 关
    0xE7AAA4  // 停
};
static uint8_t rf_tick = 0;

static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t *edata,
                                        void *user_data) {
  BaseType_t high_task_awoken = pdFALSE;
  uint8_t data = 0;
  xQueueSendFromISR(rf_data_queue, &data, &high_task_awoken);
  // return whether we need to yield at the end of ISR
  return (high_task_awoken == pdTRUE);
}

static void rf_send_task(void *args) {
  while (1) {
    uint8_t index;
    xQueueReceive(rf_send_queue, &index, portMAX_DELAY);
    for (uint8_t i = 0; i < RF_SEND_REPEAT; i++) {
      gpio_set_level(RF_DATA_GPIO, 1);
      ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
      ESP_ERROR_CHECK(gptimer_start(gptimer));

      uint8_t data;
      while (rf_tick < 128) {
        xQueueReceive(rf_data_queue, &data, portMAX_DELAY);
        rf_tick++;
        if (rf_tick == 1) {
          gpio_set_level(RF_DATA_GPIO, 0);
          continue;
        }
        if (rf_tick == 32) {
          gpio_set_level(RF_DATA_GPIO, 1);
          continue;
        }
        if (rf_tick == 128) {
          break;
        }
        if (rf_tick > 32) {
          if (rf_tick % 4 == 0) {
            gpio_set_level(RF_DATA_GPIO, 1);
            continue;
          }
          uint8_t data_bit = (rf_tick - 28) / 4;
          if ((rf_data[index] >> (24 - data_bit)) & 1) {
            if (rf_tick % 4 == 3) {
              gpio_set_level(RF_DATA_GPIO, 0);
            }
          } else {
            if (rf_tick % 4 == 1) {
              gpio_set_level(RF_DATA_GPIO, 0);
            }
          }
        }
      }
      ESP_ERROR_CHECK(gptimer_stop(gptimer));
      gpio_set_level(RF_DATA_GPIO, 0);
      rf_tick = 0;
    }
  }
}

void rf_send_open(void) {
  ESP_LOGI("RF", "open");
  uint8_t index = 0;
  xQueueSend(rf_send_queue, &index, 0);
}

void rf_send_close(void) {
  ESP_LOGI("RF", "close");
  uint8_t index = 1;
  xQueueSend(rf_send_queue, &index, 0);
}

void rf_send_stop(void) {
  ESP_LOGI("RF", "stop");
  uint8_t index = 2;
  xQueueSend(rf_send_queue, &index, 0);
}

void rf_control_init(void) {
  // 初始化编码输出引脚
  gpio_reset_pin(RF_DATA_GPIO);
  gpio_set_direction(RF_DATA_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(RF_DATA_GPIO, GPIO_PULLDOWN_ONLY);

  // 初始化定时器
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000,
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = timer_on_alarm_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  gptimer_alarm_config_t alarm_config = {
      .reload_count = 0,
      .alarm_count = RF_PAULSE_WIDTH_MIN,
      .flags.auto_reload_on_alarm = true,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

  // 创建发送任务
  rf_send_queue = xQueueCreate(1, sizeof(uint8_t));
  rf_data_queue = xQueueCreate(1, sizeof(uint8_t));
  xTaskCreate(rf_send_task, "rf_send_task", 2048, NULL, 5, NULL);
}