#ifndef _H_BLE_
#define _H_BLE_

#define IBEACON_AVG_WINDOW 5       // iBeacon设备信号接收平均窗口
#define IBEACON_ENTER_RSSI 93      // iBeacon进入判断信号强度
#define IBEACON_LEAVE_RSSI 96      // iBeacon离开判断信号强度
#define IBEACON_LEAVE_TIMEOUT 3000 // iBeacon离开判断信号强度超时判断

void ble_init(void);
#endif