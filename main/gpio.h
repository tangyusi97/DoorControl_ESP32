#ifndef _H_GPIO_
#define _H_GPIO_

#define BLINK_GPIO 2
#define FINGER_TOUCH_GPIO 4
#define DOOR_OPEN_GPIO 25
#define DOOR_CLOSE_GPIO 26
#define DOOR_STOP_GPIO 27
#define KEY_DURATION 500      // 遥控器按键时长
#define OPEN_DURATION 5000    // 开一下门的等待时长
#define CONTROL_INTERVAL 1000 // 开一下门的等待时长

void led_on(void);
void led_off(void);
void gpio_init(void);

void door_open(void);
void door_stop(void);
void door_close(void);
void door_open_and_close(void);

#endif