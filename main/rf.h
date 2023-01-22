#ifndef _H_RF_
#define _H_RF_

#define RF_PAULSE_WIDTH_MIN 570 // 遥控编码最小脉宽us
#define RF_DATA_GPIO 22         // 编码输出引脚
#define RF_SEND_REPEAT 10       // 射频信号重复发送次数

void rf_send_open(void);
void rf_send_close(void);
void rf_send_stop(void);

void rf_control_init(void);

#endif