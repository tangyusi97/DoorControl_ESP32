#include "gpio.h"
#include "esp_log.h"
#include "finger.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdint.h>

static uint8_t can_start = 0; // 开始执行开一下门
static uint8_t is_interr = 0; // 开一下门过程是否被打断

static QueueHandle_t key_press_queue;
static QueueHandle_t beep_queue;

void led_on(void) { gpio_set_level(LED_GPIO, 1); }

void led_off(void) { gpio_set_level(LED_GPIO, 0); }

void beep_ok(void) {
  uint8_t type = 0;
  xQueueSend(beep_queue, &type, 0);
}

void beep_error(void) {
  uint8_t type = 1;
  xQueueSend(beep_queue, &type, 0);
}

// 设置蜂鸣器音高（hz）
static void beep_set_freq(uint16_t hz) {
  ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, hz * 2));
}

// 设置蜂鸣器占空比（%）
static void beep_set_duty(uint8_t duty) {
  ESP_ERROR_CHECK(
      ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 0x1FFF * duty / 100));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0));
}

static void beep_task(void *args) {
  uint8_t type = 0;
  while (1) {
    xQueueReceive(beep_queue, &type, portMAX_DELAY);
    if (type == 0) {
      // 成功
      beep_set_freq(880);
      beep_set_duty(85);
      vTaskDelay(150 / portTICK_PERIOD_MS);
      beep_set_freq(987);
      vTaskDelay(150 / portTICK_PERIOD_MS);
      beep_set_freq(1318);
      vTaskDelay(300 / portTICK_PERIOD_MS);
      beep_set_duty(0);
    } else {
      // 失败
      beep_set_freq(1108);
      beep_set_duty(75);
      vTaskDelay(150 / portTICK_PERIOD_MS);
      beep_set_duty(0);
      vTaskDelay(20 / portTICK_PERIOD_MS);
      beep_set_duty(75);
      vTaskDelay(150 / portTICK_PERIOD_MS);
      beep_set_duty(0);
    }
  }
}

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

void door_open_and_close(void) { can_start = 1; }

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

static void ledc_init(void) {
  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                    .timer_num = LEDC_TIMER_0,
                                    .duty_resolution = LEDC_TIMER_13_BIT,
                                    .freq_hz = 2048,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                        .channel = LEDC_CHANNEL_0,
                                        .timer_sel = LEDC_TIMER_0,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = BEEP_GPIO,
                                        .duty = 0,
                                        .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void gpio_init(void) {
  // 初始化LED灯
  gpio_reset_pin(LED_GPIO);
  gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(LED_GPIO, GPIO_PULLDOWN_ONLY);

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

  // 初始化BEEP
  ledc_init();

  key_press_queue = xQueueCreate(2, sizeof(uint8_t));
  beep_queue = xQueueCreate(2, sizeof(uint8_t));
  xTaskCreate(key_press_task, "key_press_task", 2048, NULL, 10, NULL);
  xTaskCreate(beep_task, "beep_task", 2048, NULL, 10, NULL);
  xTaskCreate(door_open_and_close_task, "door_open_and_close_task", 2048, NULL,
              14, NULL);
  xTaskCreate(detect_finger_task, "detect_finger_task", 2048, NULL, 12, NULL);
}
