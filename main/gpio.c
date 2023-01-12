#include "gpio.h"
#include "esp_log.h"
#include "finger.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdint.h>

static uint8_t can_start = 0;      // 开始执行开一下门
static uint8_t is_interr = 0;      // 开一下门过程是否被打断

static QueueHandle_t key_press_queue;

void led_on(void) { gpio_set_level(BLINK_GPIO, 1); }

void led_off(void) { gpio_set_level(BLINK_GPIO, 0); }

static void key_press(uint8_t gpio) {
  uint8_t key = gpio;
  xQueueSend(key_press_queue, &key, 0);
}

void door_open(void) {
  is_interr = 1;
  key_press(DOOR_OPEN_GPIO);
}

void door_stop(void) {
  is_interr = 1;
  key_press(DOOR_STOP_GPIO);
}

void door_close(void) {
  is_interr = 1;
  key_press(DOOR_CLOSE_GPIO);
}

void door_open_and_close(void) {
  can_start = 1;
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

static void door_open_and_close_task(void *args) {
  while (1) {
    while (!can_start) {
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    is_interr = 0;
    key_press(DOOR_OPEN_GPIO);
    vTaskDelay(OPEN_DURATION / portTICK_PERIOD_MS);

    if (is_interr) {
      is_interr = 0;
      can_start = 0;
      continue;
    }
    key_press(DOOR_STOP_GPIO);
    vTaskDelay(STOP_DURATION / portTICK_PERIOD_MS);

    if (is_interr) {
      is_interr = 0;
      can_start = 0;
      continue;
    }
    key_press(DOOR_CLOSE_GPIO);
    can_start = 0;
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
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

void gpio_init(void) {
  // 初始化LED灯
  gpio_reset_pin(BLINK_GPIO);
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(BLINK_GPIO, GPIO_PULLDOWN_ONLY);

  // 初始化 开
  gpio_reset_pin(DOOR_OPEN_GPIO);
  gpio_set_direction(DOOR_OPEN_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(DOOR_OPEN_GPIO, GPIO_PULLDOWN_ONLY);

  // 初始化 停
  gpio_reset_pin(DOOR_STOP_GPIO);
  gpio_set_direction(DOOR_STOP_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(DOOR_STOP_GPIO, GPIO_PULLDOWN_ONLY);

  // 初始化 关
  gpio_reset_pin(DOOR_CLOSE_GPIO);
  gpio_set_direction(DOOR_CLOSE_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(DOOR_CLOSE_GPIO, GPIO_PULLDOWN_ONLY);

  key_press_queue = xQueueCreate(2, sizeof(uint8_t));
  xTaskCreate(key_press_task, "key_press_task", 2048, NULL, 10, NULL);
  xTaskCreate(door_open_and_close_task, "door_open_and_close_task", 2048, NULL,
              14, NULL);
  xTaskCreate(detect_finger_task, "detect_finger_task", 2048, NULL, 12, NULL);
}
