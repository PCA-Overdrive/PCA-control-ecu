#ifndef APP_AUTO_EXIT_SERVICE_INTERNAL_H
#define APP_AUTO_EXIT_SERVICE_INTERNAL_H

#include "App_Types.h"

#define APP_AUTO_EXIT_SERVICE_PERIOD_MS 10u

#define APP_AUTO_EXIT_DRIVE_STOP       127u
#define APP_AUTO_EXIT_STEER_CENTER     127u

#define APP_AUTO_EXIT_DRIVE_FORWARD    80u
#define APP_AUTO_EXIT_DRIVE_REVERSE    200u

#define APP_AUTO_EXIT_STEER_LEFT       0u
#define APP_AUTO_EXIT_STEER_RIGHT      255u

#define APP_AUTO_EXIT_SHIFT_STOP_MS    300u
#define APP_AUTO_EXIT_FORWARD_1_MS     3000u
#define APP_AUTO_EXIT_TURN_1_MS        2500u
#define APP_AUTO_EXIT_REVERSE_1_MS     1800u
#define APP_AUTO_EXIT_TURN_2_MS        2700u
#define APP_AUTO_EXIT_REVERSE_TURN_MS  2400u
#define APP_AUTO_EXIT_TURN_3_MS        3000u
#define APP_AUTO_EXIT_FORWARD_2_MS     500u
#define APP_AUTO_EXIT_FINAL_STOP_MS    700u
#define APP_AUTO_EXIT_MIN_STEP_MS      100u

#define APP_AUTO_EXIT_START_STOP_MS          300u

#define APP_AUTO_EXIT_FRONT_HARD_STOP_MM     250u
#define APP_AUTO_EXIT_FRONT_BLOCKED_MM       350u
#define APP_AUTO_EXIT_REAR_HARD_STOP_MM      250u

#define APP_AUTO_EXIT_SIDE_SAFE_MM           450u
#define APP_AUTO_EXIT_SIDE_MIN_MM            220u
#define APP_AUTO_EXIT_SIDE_TILT_DIFF_MM      50u

#define APP_AUTO_EXIT_SIDE_FRONT_CAUTION_MM  450u
#define APP_AUTO_EXIT_SIDE_REAR_CAUTION_MM   400u
#define APP_AUTO_EXIT_SIDE_BLOCKED_MM        180u

#define APP_AUTO_EXIT_AVOID_ESCAPE_MIN_MS    250u
#define APP_AUTO_EXIT_AVOID_ESCAPE_SHORT_MS  700u
#define APP_AUTO_EXIT_AVOID_ESCAPE_LONG_MS   1500u

#define APP_AUTO_EXIT_AVOID_REALIGN_SHORT_MS 700u
#define APP_AUTO_EXIT_AVOID_REALIGN_LONG_MS  1800u

#define APP_AUTO_EXIT_REALIGN_LEFT_STEER     64u
#define APP_AUTO_EXIT_REALIGN_RIGHT_STEER    191u

#define APP_AUTO_EXIT_AVOID_ESCAPE_FORWARD_RATIO_PERCENT  60u
#define APP_AUTO_EXIT_AVOID_REALIGN_FORWARD_RATIO_PERCENT 60u

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

const AppAutoExitMotionStep *AppAutoExitProfile_Get(AppAutoExitDirection direction,
                                                    uint32 *profileCount);

boolean AppAutoExitPlanner_IsStepSafetyDanger(const AppAutoExitMotionStep *step);

AppAutoExitStrategy AppAutoExitPlanner_SelectStrategy(AppAutoExitDirection exitDirection,
                                                      AppAutoExitAvoidPlan *avoidPlan);

uint8 AppAutoExitPlanner_GetEscapeSteer(AppAutoExitDirection exitDirection);
uint8 AppAutoExitPlanner_GetRealignSteer(AppAutoExitDirection exitDirection);

boolean AppAutoExitPlanner_IsOppositeSideDangerDuringAvoid(AppAutoExitDirection exitDirection);

uint32 AppAutoExitPlanner_CalcFirstStepReductionMs(uint32 escapeMs,
                                                   uint32 realignMs);

#endif /* APP_AUTO_EXIT_SERVICE_INTERNAL_H */
