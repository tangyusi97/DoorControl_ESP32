#include "util.h"

void print_byte(uint8_t *data, uint8_t len) {
  for (int i = 0; i < len; i++) {
    printf("%02x ", data[i]);
  }
  printf("\n");
}
