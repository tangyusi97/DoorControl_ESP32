#ifndef _H_BEEP_
#define _H_BEEP_

#define LED_GPIO 2
#define BEEP_GPIO 2

void led_on(void);
void led_off(void);

void beep_ok(void);
void beep_error(void);

void beep_init(void);

#endif