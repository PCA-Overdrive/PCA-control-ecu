#ifndef APP_AUTO_EXIT_TYPES_H
#define APP_AUTO_EXIT_TYPES_H

#include "App_Types.h"

typedef enum
{
    APP_AUTO_EXIT_DIR_STRAIGHT = 0,
    APP_AUTO_EXIT_DIR_LEFT,
    APP_AUTO_EXIT_DIR_RIGHT
} AppAutoExitDirection;

typedef enum
{
    APP_AUTO_EXIT_STRATEGY_NORMAL = 0,
    APP_AUTO_EXIT_STRATEGY_AVOID_AND_RESUME,
    APP_AUTO_EXIT_STRATEGY_BLOCKED
} AppAutoExitStrategy;

typedef enum
{
    APP_AUTO_EXIT_AVOID_NONE = 0,
    APP_AUTO_EXIT_AVOID_SHORT,
    APP_AUTO_EXIT_AVOID_LONG
} AppAutoExitAvoidLevel;

typedef enum
{
    APP_AUTO_EXIT_STATUS_IDLE = 0x00u,
    APP_AUTO_EXIT_STATUS_IN_PROGRESS = 0x01u,
    APP_AUTO_EXIT_STATUS_COMPLETE = 0x02u,
    APP_AUTO_EXIT_STATUS_STOPPED = 0x03u,
    APP_AUTO_EXIT_STATUS_BLOCKED = APP_AUTO_EXIT_STATUS_STOPPED
} AppAutoExitStatus;

typedef struct
{
    uint8 driveCmd;
    uint8 steeringCmd;
    uint32 durationMs;
} AppAutoExitMotionStep;

typedef struct
{
    uint32 escapeMs;
    uint32 realignMs;
} AppAutoExitAvoidPlan;

#endif /* APP_AUTO_EXIT_TYPES_H */
