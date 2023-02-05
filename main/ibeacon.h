#ifndef _H_IBEACON_
#define _H_IBEACON_

#include <stdint.h>

#define IBEACONS_NUM 2             // iBeacon设备的数量
#define IBEACON_AVG_WINDOW 5       // iBeacon设备信号接收平均窗口
#define IBEACON_ENTER_RSSI 93      // iBeacon进入判断信号强度
#define IBEACON_LEAVE_RSSI 96      // iBeacon离开判断信号强度
#define IBEACON_LEAVE_TIMEOUT 3000 // iBeacon离开判断信号强度超时判断

uint8_t check_ibeacon(uint8_t *addr, uint8_t rssi);
void ibeacon_init(void);
#endif