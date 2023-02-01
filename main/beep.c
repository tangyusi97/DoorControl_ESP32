#include "beep.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

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
      beep_set_duty(75);
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

void beep_init(void) {
  gpio_reset_pin(LED_GPIO);
  gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(LED_GPIO, GPIO_PULLDOWN_ONLY);

  ledc_init();

  beep_queue = xQueueCreate(2, sizeof(uint8_t));

  xTaskCreate(beep_task, "beep_task", 2048, NULL, 10, NULL);
}