#ifndef _H_FINGER_
#define _H_FINGER_

#define FINGER_TXD 19
#define FINGER_RXD 18
#define FINGER_TOUCH_GPIO 21

#define FINGER_UART_PORT_NUM 2
#define FINGER_UART_BAUD_RATE 57600
#define FINGER_UART_BUF_SIZE 128

#define FINGER_TIMEOUT 10000             // 指纹超时时间
#define FINGER_SUCCEED_ACTION_DELAY 1000 // 指纹识别成功后响应延时

void finger_init(void);
void finger_verify(void);
void finger_enroll(void);

#endif