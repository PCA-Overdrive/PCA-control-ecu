#include "App_AutoExitService_Internal.h"

#include "App_RxService.h"

typedef struct
{
    uint16 frontMm;
    uint16 rearMm;
    uint16 minMm;
    boolean isSafe;
    boolean isFrontCloser;
    boolean isRearCloser;
} AppAutoExitSideInfo;

typedef struct
{
    AppAutoExitSideInfo exitSide;
    AppAutoExitSideInfo oppositeSide;
    uint16 exitFrontCornerMm;
} AppAutoExitSideContext;

static boolean AppAutoExitPlanner_IsAnyDistanceBelow(const AppUltrasonicState *ultrasonic,
                                                     const AppPdwDirection *directions,
                                                     uint32 directionCount,
                                                     uint16 thresholdMm)
{
    uint32 index;

    for(index = 0u; index < directionCount; index++)
    {
        if(ultrasonic->distanceMm[directions[index]] < thresholdMm)
        {
            return TRUE;
        }
    }

    return FALSE;
}

static AppAutoExitSideInfo AppAutoExitPlanner_MakeSideInfo(uint16 frontMm,
                                                           uint16 rearMm)
{
    AppAutoExitSideInfo info;
    sint32 diffMm;

    info.frontMm = frontMm;
    info.rearMm = rearMm;
    info.minMm = (frontMm < rearMm) ? frontMm : rearMm;
    info.isSafe = ((frontMm > APP_AUTO_EXIT_SIDE_SAFE_MM) &&
                   (rearMm > APP_AUTO_EXIT_SIDE_SAFE_MM)) ? TRUE : FALSE;

    diffMm = (sint32)frontMm - (sint32)rearMm;

    if(diffMm < -(sint32)APP_AUTO_EXIT_SIDE_TILT_DIFF_MM)
    {
        info.isFrontCloser = TRUE;
        info.isRearCloser = FALSE;
    }
    else if(diffMm > (sint32)APP_AUTO_EXIT_SIDE_TILT_DIFF_MM)
    {
        info.isFrontCloser = FALSE;
        info.isRearCloser = TRUE;
    }
    else
    {
        info.isFrontCloser = FALSE;
        info.isRearCloser = FALSE;
    }

    return info;
}

static AppAutoExitAvoidLevel AppAutoExitPlanner_GetAvoidLevel(const AppAutoExitSideInfo *exitSide)
{
    if(exitSide == 0)
    {
        return APP_AUTO_EXIT_AVOID_LONG;
    }

    if(exitSide->minMm < APP_AUTO_EXIT_SIDE_BLOCKED_MM)
    {
        return APP_AUTO_EXIT_AVOID_LONG;
    }

    if(exitSide->frontMm < APP_AUTO_EXIT_SIDE_FRONT_CAUTION_MM)
    {
        return APP_AUTO_EXIT_AVOID_LONG;
    }

    if(exitSide->rearMm < APP_AUTO_EXIT_SIDE_REAR_CAUTION_MM)
    {
        return APP_AUTO_EXIT_AVOID_SHORT;
    }

    if((exitSide->isFrontCloser == TRUE) &&
       (exitSide->frontMm < APP_AUTO_EXIT_SIDE_SAFE_MM))
    {
        return APP_AUTO_EXIT_AVOID_LONG;
    }

    if((exitSide->isRearCloser == TRUE) &&
       (exitSide->rearMm < APP_AUTO_EXIT_SIDE_SAFE_MM))
    {
        return APP_AUTO_EXIT_AVOID_SHORT;
    }

    return APP_AUTO_EXIT_AVOID_NONE;
}

static AppAutoExitSideContext AppAutoExitPlanner_MakeSideContext(const AppUltrasonicState *ultrasonic,
                                                                 AppAutoExitDirection exitDirection)
{
    AppAutoExitSideContext context;

    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        context.exitSide = AppAutoExitPlanner_MakeSideInfo(
            ultrasonic->distanceMm[APP_PDW_DIR_LEFT_FRONT],
            ultrasonic->distanceMm[APP_PDW_DIR_LEFT_BEHIND]);

        context.oppositeSide = AppAutoExitPlanner_MakeSideInfo(
            ultrasonic->distanceMm[APP_PDW_DIR_RIGHT_FRONT],
            ultrasonic->distanceMm[APP_PDW_DIR_RIGHT_BEHIND]);

        context.exitFrontCornerMm = ultrasonic->distanceMm[APP_PDW_DIR_FRONT_LEFT];
    }
    else
    {
        context.exitSide = AppAutoExitPlanner_MakeSideInfo(
            ultrasonic->distanceMm[APP_PDW_DIR_RIGHT_FRONT],
            ultrasonic->distanceMm[APP_PDW_DIR_RIGHT_BEHIND]);

        context.oppositeSide = AppAutoExitPlanner_MakeSideInfo(
            ultrasonic->distanceMm[APP_PDW_DIR_LEFT_FRONT],
            ultrasonic->distanceMm[APP_PDW_DIR_LEFT_BEHIND]);

        context.exitFrontCornerMm = ultrasonic->distanceMm[APP_PDW_DIR_FRONT_RIGHT];
    }

    return context;
}

static void AppAutoExitPlanner_SetAvoidPlan(AppAutoExitAvoidPlan *avoidPlan,
                                            AppAutoExitAvoidLevel level)
{
    if(avoidPlan == 0)
    {
        return;
    }

    if(level == APP_AUTO_EXIT_AVOID_LONG)
    {
        avoidPlan->escapeMs = APP_AUTO_EXIT_AVOID_ESCAPE_LONG_MS;
        avoidPlan->realignMs = APP_AUTO_EXIT_AVOID_REALIGN_LONG_MS;
    }
    else if(level == APP_AUTO_EXIT_AVOID_SHORT)
    {
        avoidPlan->escapeMs = APP_AUTO_EXIT_AVOID_ESCAPE_SHORT_MS;
        avoidPlan->realignMs = APP_AUTO_EXIT_AVOID_REALIGN_SHORT_MS;
    }
    else
    {
        avoidPlan->escapeMs = 0u;
        avoidPlan->realignMs = 0u;
    }
}

boolean AppAutoExitPlanner_IsStepSafetyDanger(const AppAutoExitMotionStep *step)
{
    AppUltrasonicState ultrasonic;
    static const AppPdwDirection frontDirections[] =
    {
        APP_PDW_DIR_FRONT,
        APP_PDW_DIR_FRONT_LEFT,
        APP_PDW_DIR_FRONT_RIGHT
    };
    static const AppPdwDirection rearDirections[] =
    {
        APP_PDW_DIR_BEHIND,
        APP_PDW_DIR_BEHIND_LEFT,
        APP_PDW_DIR_BEHIND_RIGHT
    };

    if(step == 0)
    {
        return FALSE;
    }

    if(AppRxService_GetUltrasonicState(&ultrasonic) != pdPASS)
    {
        return FALSE;
    }

    if(step->driveCmd < APP_AUTO_EXIT_DRIVE_STOP)
    {
        return AppAutoExitPlanner_IsAnyDistanceBelow(&ultrasonic,
                                                     frontDirections,
                                                     (uint32)(sizeof(frontDirections) / sizeof(frontDirections[0])),
                                                     APP_AUTO_EXIT_FRONT_HARD_STOP_MM);
    }

    if(step->driveCmd > APP_AUTO_EXIT_DRIVE_STOP)
    {
        return AppAutoExitPlanner_IsAnyDistanceBelow(&ultrasonic,
                                                     rearDirections,
                                                     (uint32)(sizeof(rearDirections) / sizeof(rearDirections[0])),
                                                     APP_AUTO_EXIT_REAR_HARD_STOP_MM);
    }

    return FALSE;
}

AppAutoExitStrategy AppAutoExitPlanner_SelectStrategy(AppAutoExitDirection exitDirection,
                                                      AppAutoExitAvoidPlan *avoidPlan)
{
    AppUltrasonicState ultrasonic;
    AppAutoExitSideContext sideContext;
    AppAutoExitAvoidLevel avoidLevel;

    AppAutoExitPlanner_SetAvoidPlan(avoidPlan, APP_AUTO_EXIT_AVOID_NONE);

    if(exitDirection == APP_AUTO_EXIT_DIR_STRAIGHT)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    if(AppRxService_GetUltrasonicState(&ultrasonic) != pdPASS)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    sideContext = AppAutoExitPlanner_MakeSideContext(&ultrasonic,
                                                     exitDirection);

    if(ultrasonic.distanceMm[APP_PDW_DIR_FRONT] < APP_AUTO_EXIT_FRONT_BLOCKED_MM)
    {
        return APP_AUTO_EXIT_STRATEGY_BLOCKED;
    }

    if(sideContext.exitFrontCornerMm < APP_AUTO_EXIT_FRONT_BLOCKED_MM)
    {
        return APP_AUTO_EXIT_STRATEGY_BLOCKED;
    }

    avoidLevel = AppAutoExitPlanner_GetAvoidLevel(&sideContext.exitSide);

    if((avoidLevel == APP_AUTO_EXIT_AVOID_NONE) &&
       (sideContext.exitSide.isSafe == TRUE))
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    if(sideContext.oppositeSide.isSafe == TRUE)
    {
        if(avoidLevel == APP_AUTO_EXIT_AVOID_NONE)
        {
            avoidLevel = APP_AUTO_EXIT_AVOID_SHORT;
        }

        AppAutoExitPlanner_SetAvoidPlan(avoidPlan, avoidLevel);
        return APP_AUTO_EXIT_STRATEGY_AVOID_AND_RESUME;
    }

    return APP_AUTO_EXIT_STRATEGY_BLOCKED;
}

uint8 AppAutoExitPlanner_GetEscapeSteer(AppAutoExitDirection exitDirection)
{
    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        return APP_AUTO_EXIT_STEER_RIGHT;
    }

    return APP_AUTO_EXIT_STEER_LEFT;
}

uint8 AppAutoExitPlanner_GetRealignSteer(AppAutoExitDirection exitDirection)
{
    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        return APP_AUTO_EXIT_REALIGN_LEFT_STEER;
    }

    return APP_AUTO_EXIT_REALIGN_RIGHT_STEER;
}

static boolean AppAutoExitPlanner_IsAvoidSideDanger(const AppUltrasonicState *ultrasonic,
                                                    AppPdwDirection sideFrontDirection,
                                                    AppPdwDirection sideRearDirection,
                                                    AppPdwDirection frontCornerDirection)
{
    if(ultrasonic->distanceMm[sideFrontDirection] < APP_AUTO_EXIT_SIDE_MIN_MM)
    {
        return TRUE;
    }

    if(ultrasonic->distanceMm[sideRearDirection] < APP_AUTO_EXIT_SIDE_MIN_MM)
    {
        return TRUE;
    }

    if(ultrasonic->distanceMm[frontCornerDirection] < APP_AUTO_EXIT_FRONT_HARD_STOP_MM)
    {
        return TRUE;
    }

    return FALSE;
}

boolean AppAutoExitPlanner_IsOppositeSideDangerDuringAvoid(AppAutoExitDirection exitDirection)
{
    AppUltrasonicState ultrasonic;

    if(AppRxService_GetUltrasonicState(&ultrasonic) != pdPASS)
    {
        return FALSE;
    }

    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        return AppAutoExitPlanner_IsAvoidSideDanger(&ultrasonic,
                                                    APP_PDW_DIR_RIGHT_FRONT,
                                                    APP_PDW_DIR_RIGHT_BEHIND,
                                                    APP_PDW_DIR_FRONT_RIGHT);
    }

    return AppAutoExitPlanner_IsAvoidSideDanger(&ultrasonic,
                                                APP_PDW_DIR_LEFT_FRONT,
                                                APP_PDW_DIR_LEFT_BEHIND,
                                                APP_PDW_DIR_FRONT_LEFT);
}

uint32 AppAutoExitPlanner_CalcFirstStepReductionMs(uint32 escapeMs,
                                                   uint32 realignMs)
{
    uint32 reductionMs;

    reductionMs =
        ((escapeMs * APP_AUTO_EXIT_AVOID_ESCAPE_FORWARD_RATIO_PERCENT) / 100u) +
        ((realignMs * APP_AUTO_EXIT_AVOID_REALIGN_FORWARD_RATIO_PERCENT) / 100u);

    if(reductionMs >= APP_AUTO_EXIT_FORWARD_1_MS)
    {
        return APP_AUTO_EXIT_FORWARD_1_MS - APP_AUTO_EXIT_MIN_STEP_MS;
    }

    return reductionMs;
}
