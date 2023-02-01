#ifndef _H_BOARD_
#define _H_BOARD_

#define BOARD_OPEN_GPIO 27
#define BOARD_CLOSE_GPIO 26
#define BOARD_STOP_GPIO 25
#define KEY_DURATION 600   // 遥控器按键时长

#include <stdint.h>

void board_key_press(uint8_t key);
void board_init(void);

#endif