#ifndef __COMMON_H
#define __COMMON_H

#include "sys.h"

/*============================================================================
 * 系统模式定义
 *============================================================================*/
typedef enum {
    MODE_MANUAL    = 0,   /* 手动模式 */
    MODE_AUTO      = 1,   /* 自动模式 */
    MODE_BLUETOOTH = 2    /* 蓝牙模式 */
} SystemMode_t;

/* 蓝牙子模式 */
typedef enum {
    WORK_MODE_MANUAL = 0,   /* 蓝牙手动 */
    WORK_MODE_AUTO   = 1    /* 蓝牙自动 */
} BTWorkMode_t;

/*============================================================================
 * 通用工具宏
 *============================================================================*/

/* 范围限幅 */
#define LIMIT(x, min, max)  (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

/* 数组元素个数 */
#define ARRAY_SIZE(arr)     (sizeof(arr) / sizeof(arr[0]))

#endif