#include "control.h"
#include "board.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "rf.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

// bit0:开始执行；bit1：打断
static EventGroupHandle_t open_and_close_event;

static void door_move(DOOR_ACTION action) {
#ifdef ENABLE_RF
  rf_send(action);
#else
  board_key_press(action);
#endif
}

void door_control(DOOR_ACTION action) {
  if (action < OPEN_AND_CLOSE) {
    xEventGroupSetBits(open_and_close_event, 0b10);
    door_move(action);
  } else {
    xEventGroupSetBits(open_and_close_event, 0b01);
  }
}

static void door_open_and_close_task(void *args) {
  while (1) {
    xEventGroupWaitBits(open_and_close_event, 0b01, pdFALSE, pdFALSE,
                        portMAX_DELAY);
    xEventGroupClearBits(open_and_close_event, 0b10);

    door_move(OPEN);

    if (xEventGroupWaitBits(open_and_close_event, 0b10, pdFALSE, pdFALSE,
                            OPEN_DURATION / portTICK_PERIOD_MS) & 0b10) {
      goto end;
    }

    door_move(STOP);

    if (xEventGroupWaitBits(open_and_close_event, 0b10, pdFALSE, pdFALSE,
                            STOP_DURATION / portTICK_PERIOD_MS) & 0b10) {
      goto end;
    }

    door_move(CLOSE);

    if (xEventGroupWaitBits(open_and_close_event, 0b10, pdFALSE, pdFALSE,
                            OPEN_DURATION / portTICK_PERIOD_MS) & 0b10) {
      goto end;
    }

  end:
    xEventGroupClearBits(open_and_close_event, 0b11);
  }
}

void control_init(void) {
#ifdef ENABLE_RF
  rf_control_init();
#else
  board_init();
#endif

  open_and_close_event = xEventGroupCreate();
  xTaskCreate(door_open_and_close_task, "door_open_and_close_task", 2048, NULL,
              14, NULL);
}
