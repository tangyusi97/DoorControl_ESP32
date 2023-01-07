#ifndef _H_FINGER_
#define _H_FINGER_

#define FINGER_TXD (17)
#define FINGER_RXD (16)

#define FINGER_UART_PORT_NUM (2)
#define FINGER_UART_BAUD_RATE (57600)

#define FINGER_UART_BUF_SIZE (128)

void finger_init(void);
void finger_verify(void);
void finger_enroll(void);

#endif