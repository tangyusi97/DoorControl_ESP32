#include "board.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"


static QueueHandle_t key_press_queue;
static uint8_t key_gpio[] = {
    BOARD_OPEN_GPIO,
    BOARD_CLOSE_GPIO,
    BOARD_STOP_GPIO,
};

void board_key_press(uint8_t key) {
  xQueueSend(key_press_queue, &key_gpio[key], 0);
}

static void key_press_task(void *args) {
  uint8_t key = 0;
  while (1) {
    xQueueReceive(key_press_queue, &key, portMAX_DELAY);
    ESP_LOGI("GPIO", "Key pressed: %d", key);
    gpio_set_level(key, 1);
    vTaskDelay(KEY_DURATION / portTICK_PERIOD_MS);
    gpio_set_level(key, 0);
  }
}

void board_init(void) {

  // 初始化 开
  gpio_reset_pin(BOARD_OPEN_GPIO);
  gpio_set_direction(BOARD_OPEN_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(BOARD_OPEN_GPIO, GPIO_PULLDOWN_ONLY);

  // 初始化 停
  gpio_reset_pin(BOARD_STOP_GPIO);
  gpio_set_direction(BOARD_STOP_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(BOARD_STOP_GPIO, GPIO_PULLDOWN_ONLY);

  // 初始化 关
  gpio_reset_pin(BOARD_CLOSE_GPIO);
  gpio_set_direction(BOARD_CLOSE_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(BOARD_CLOSE_GPIO, GPIO_PULLDOWN_ONLY);

  key_press_queue = xQueueCreate(2, sizeof(uint8_t));
  xTaskCreate(key_press_task, "key_press_task", 2048, NULL, 10, NULL);
}