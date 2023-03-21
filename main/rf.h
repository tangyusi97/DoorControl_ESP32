#ifndef _H_RF_
#define _H_RF_

#define RF_PAULSE_WIDTH_MIN 570 // 遥控编码最小脉宽us
#define RF_DATA_GPIO 23         // 编码输出引脚
#define RF_SEND_REPEAT 8        // 射频信号重复发送次数

#include <stdint.h>

void rf_send(uint8_t index);

void rf_control_init(void);

#endif