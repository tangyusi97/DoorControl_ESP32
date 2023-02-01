#ifndef _H_CONTROL_
#define _H_CONTROL_

#define ENABLE_RF

#define OPEN_DURATION 5000 // 开一下门的开门时长
#define STOP_DURATION 1500 // 开一下门的停止时长

typedef enum DOOR_ACTION{
    OPEN, CLOSE, STOP, OPEN_AND_CLOSE
} DOOR_ACTION;

void door_control(DOOR_ACTION action);
void control_init(void);

#endif