#include "control.h"
#include "board.h"
#include "rf.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static uint8_t can_start = 0; // 开始执行开一下门
static uint8_t is_interr = 0; // 开一下门过程是否被打断

static void door_move(DOOR_ACTION action) {
#ifdef ENABLE_RF
  rf_send(action);
#else
  board_key_press(action);
#endif
}

void door_control(DOOR_ACTION action) {
  if (action < OPEN_AND_CLOSE) {
    is_interr = 1;
    door_move(action);
  } else {
    can_start = 1;
  }
}

static void door_open_and_close_task(void *args) {
  while (1) {
    while (!can_start) {
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    is_interr = 0;
    door_move(OPEN);
    vTaskDelay(OPEN_DURATION / portTICK_PERIOD_MS);

    if (is_interr) {
      is_interr = 0;
      can_start = 0;
      continue;
    }
    door_move(STOP);
    vTaskDelay(STOP_DURATION / portTICK_PERIOD_MS);

    if (is_interr) {
      is_interr = 0;
      can_start = 0;
      continue;
    }
    door_move(CLOSE);
    can_start = 0;
  }
}

void control_init(void) {
#ifdef ENABLE_RF
  rf_control_init();
#else
  board_init();
#endif
  xTaskCreate(door_open_and_close_task, "door_open_and_close_task", 2048, NULL,
              14, NULL);
}
