#include "ble.h"
#include "finger.h"
#include "gpio.h"

void app_main(void) {
  gpio_init();
  finger_init();
  ble_init();
}
