#include "util.h"
#include <stdint.h>

void print_byte(uint8_t *data, uint8_t len) {
  for (int i = 0; i < len; i++) {
    printf("%02x ", data[i]);
  }
  printf("\n");
}

uint8_t compare_byte(uint8_t *data1, uint8_t *data2, uint8_t len) {
  if (!len)
    return 0;
  while (--len && *data1 && *data1 == *data2) {
    data1++;
    data2++;
  }
  return *data1 - *data2;
}