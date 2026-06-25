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
    APP_AUTO_EXIT_SIDE_RISK_SAFE = 0,
    APP_AUTO_EXIT_SIDE_RISK_CRITICAL,
    APP_AUTO_EXIT_SIDE_RISK_NEAR_FRONT,
    APP_AUTO_EXIT_SIDE_RISK_NEAR_REAR,
    APP_AUTO_EXIT_SIDE_RISK_TILTED_FRONT,
    APP_AUTO_EXIT_SIDE_RISK_TILTED_REAR,
    APP_AUTO_EXIT_SIDE_RISK_NARROW_BOTH
} AppAutoExitSideRisk;

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

    uint8 escapeSteerCmd;
    uint8 realignSteerCmd;
} AppAutoExitAvoidPlan;

typedef enum
{
    APP_AUTO_EXIT_AVOID_PHASE_ESCAPE = 0,
    APP_AUTO_EXIT_AVOID_PHASE_REALIGN
} AppAutoExitAvoidPhase;

typedef enum
{
    APP_AUTO_EXIT_AVOID_OBSTACLE_CLEAR = 0,
    APP_AUTO_EXIT_AVOID_OBSTACLE_NEAR,
    APP_AUTO_EXIT_AVOID_OBSTACLE_DANGER
} AppAutoExitAvoidObstacleState;

#endif /* APP_AUTO_EXIT_TYPES_H */
