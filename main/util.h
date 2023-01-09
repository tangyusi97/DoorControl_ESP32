#ifndef _H_UTIL_
#define _H_UTIL_

#include <stdint.h>
#include <stdio.h>

void print_byte(uint8_t *data, uint8_t len);
uint8_t compare_byte(uint8_t *data1, uint8_t *data2, uint8_t len);

#endif