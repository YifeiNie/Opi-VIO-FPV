
#pragma once


#define angle_command 1
#define angle_rate_command 0

// 数组的顺序是ROLL,PITCH,YAW
typedef struct offboard {
    uint8_t data_type;
    float angle[3];   // 单位是度（°）
    float angle_rate[3];  // 单位是度每秒*（°/s）
}offboard_t;

extern offboard_t offboard;
