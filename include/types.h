#ifndef _H_TYPES_
#define _H_TYPES_

/// @brief 电源等级，根据剩余电量确定
enum V_RUN_LEVEL
{
    V_RUN_LEVEL_STOP = 0,           // 电量不足，立刻停止屏幕和主控工作
    V_RUN_LEVEL_SUPERHUNGRY = 1,    // 电量不足，小时级刷新屏幕
    V_RUN_LEVEL_LITTLEHUNGRY = 2,
    V_RUN_LEVEL_NORMAL = 3,
    V_RUN_LEVEL_YOLO = 4
};

#endif