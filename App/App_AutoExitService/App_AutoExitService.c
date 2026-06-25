#include "App_AutoExitService.h"
#include "App_AutoExitService_Internal.h"

#include "App_RxService.h"
#include "task.h"

typedef enum
{
    APP_AUTO_EXIT_STATE_IDLE = 0,
    APP_AUTO_EXIT_STATE_START_STOP,
    APP_AUTO_EXIT_STATE_SELECT_STRATEGY,
    APP_AUTO_EXIT_STATE_AVOID_ESCAPE,
    APP_AUTO_EXIT_STATE_AVOID_STOP_1,
    APP_AUTO_EXIT_STATE_AVOID_REALIGN,
    APP_AUTO_EXIT_STATE_AVOID_STOP_2,
    APP_AUTO_EXIT_STATE_RUN_PROFILE,
    APP_AUTO_EXIT_STATE_BLOCKED,
    APP_AUTO_EXIT_STATE_STOPPED
} AppAutoExitState;

static boolean g_autoExitActive = FALSE;
static boolean g_autoExitCmdValid = FALSE;
static VehicleControlCmd_t g_autoExitCmd;

static AppAutoExitState g_autoExitState = APP_AUTO_EXIT_STATE_IDLE;
static AppAutoExitDirection g_exitDirection = APP_AUTO_EXIT_DIR_STRAIGHT;
static TickType_t g_stateStartTick = 0u;

static const AppAutoExitMotionStep *g_activeProfile = 0;
static uint32 g_activeProfileCount = 0u;
static uint32 g_activeStepIndex = 0u;
static TickType_t g_stepStartTick = 0u;
static uint32 g_firstStepReductionMs = 0u;

static uint32 g_avoidEscapeMs = 0u;
static uint32 g_avoidRealignMs = 0u;
static TickType_t g_avoidEscapeStartTick = 0u;
static TickType_t g_avoidRealignStartTick = 0u;
static uint32 g_avoidEscapeElapsedMs = 0u;

static void AppAutoExitService_ResetProfile(void)
{
    g_activeProfile = 0;
    g_activeProfileCount = 0u;
    g_activeStepIndex = 0u;
    g_stepStartTick = 0u;
    g_firstStepReductionMs = 0u;
}

static void AppAutoExitService_ResetAvoidPlan(void)
{
    g_avoidEscapeMs = 0u;
    g_avoidRealignMs = 0u;
    g_avoidEscapeStartTick = 0u;
    g_avoidRealignStartTick = 0u;
    g_avoidEscapeElapsedMs = 0u;
}

static boolean AppAutoExitService_HasElapsed(TickType_t startTick,
                                             uint32 durationMs)
{
    TickType_t nowTick;

    nowTick = xTaskGetTickCount();

    return ((nowTick - startTick) >= pdMS_TO_TICKS(durationMs)) ? TRUE : FALSE;
}

static uint32 AppAutoExitService_GetElapsedMs(TickType_t startTick)
{
    TickType_t elapsedTick;

    elapsedTick = xTaskGetTickCount() - startTick;

    return (uint32)((elapsedTick * 1000u) / configTICK_RATE_HZ);
}

static void AppAutoExitService_SetCommand(uint8 driveCmd, uint8 steeringCmd)
{
    g_autoExitCmd.driveCmd = driveCmd;
    g_autoExitCmd.steeringCmd = steeringCmd;
    g_autoExitCmdValid = TRUE;
}

static void AppAutoExitService_EnterIdle(void)
{
    g_autoExitActive = FALSE;
    g_autoExitCmdValid = FALSE;
    g_autoExitState = APP_AUTO_EXIT_STATE_IDLE;
    g_exitDirection = APP_AUTO_EXIT_DIR_STRAIGHT;

    AppAutoExitService_ResetProfile();
    AppAutoExitService_ResetAvoidPlan();
}

static void AppAutoExitService_EnterTimedStopState(AppAutoExitState state)
{
    g_autoExitActive = TRUE;
    g_autoExitState = state;
    g_stateStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_STOP,
                                  APP_AUTO_EXIT_STEER_CENTER);
}

static void AppAutoExitService_EnterBlocked(void)
{
    AppAutoExitMonitor_SetResult(APP_AUTO_EXIT_STATUS_BLOCKED);
    AppAutoExitService_EnterTimedStopState(APP_AUTO_EXIT_STATE_BLOCKED);
}

static void AppAutoExitService_EnterStopped(void)
{
    AppAutoExitMonitor_SetResult(APP_AUTO_EXIT_STATUS_STOPPED);
    AppAutoExitService_EnterTimedStopState(APP_AUTO_EXIT_STATE_STOPPED);
}

static void AppAutoExitService_StopProfile(void)
{
    AppAutoExitService_EnterIdle();

    g_autoExitCmd.driveCmd = APP_AUTO_EXIT_DRIVE_STOP;
    g_autoExitCmd.steeringCmd = APP_AUTO_EXIT_STEER_CENTER;
}

static void AppAutoExitService_StartProfile(const AppAutoExitMotionStep *profile,
                                            uint32 profileCount,
                                            uint32 firstStepReductionMs)
{
    if((profile == 0) || (profileCount == 0u))
    {
        AppAutoExitService_EnterBlocked();
        return;
    }

    g_activeProfile = profile;
    g_activeProfileCount = profileCount;
    g_activeStepIndex = 0u;
    g_firstStepReductionMs = firstStepReductionMs;
    g_stepStartTick = xTaskGetTickCount();

    g_autoExitActive = TRUE;
    g_autoExitState = APP_AUTO_EXIT_STATE_RUN_PROFILE;

    AppAutoExitService_SetCommand(g_activeProfile[0].driveCmd,
                                  g_activeProfile[0].steeringCmd);
}

static void AppAutoExitService_StartProfileForDirection(AppAutoExitDirection direction,
                                                        uint32 firstStepReductionMs)
{
    const AppAutoExitMotionStep *profile;
    uint32 profileCount;

    profile = AppAutoExitProfile_Get(direction, &profileCount);
    AppAutoExitService_StartProfile(profile, profileCount, firstStepReductionMs);
}

static uint32 AppAutoExitService_GetCurrentStepDurationMs(void)
{
    uint32 durationMs;

    durationMs = g_activeProfile[g_activeStepIndex].durationMs;

    if((g_activeStepIndex == 0u) && (g_firstStepReductionMs > 0u))
    {
        if(g_firstStepReductionMs >= durationMs)
        {
            durationMs = APP_AUTO_EXIT_MIN_STEP_MS;
        }
        else
        {
            durationMs = durationMs - g_firstStepReductionMs;
        }
    }

    return durationMs;
}

static void AppAutoExitService_CompleteProfile(void)
{
    if(AppAutoExitMonitor_FinishAndValidate(g_exitDirection) == TRUE)
    {
        AppAutoExitMonitor_SetResult(APP_AUTO_EXIT_STATUS_COMPLETE);
    }
    else
    {
        AppAutoExitMonitor_SetResult(APP_AUTO_EXIT_STATUS_BLOCKED);
    }

    AppAutoExitService_StopProfile();
}

static void AppAutoExitService_ServiceProfile(void)
{
    if(g_autoExitState != APP_AUTO_EXIT_STATE_RUN_PROFILE)
    {
        return;
    }

    if((g_activeProfile == 0) || (g_activeStepIndex >= g_activeProfileCount))
    {
        AppAutoExitService_StopProfile();
        return;
    }

    if(AppAutoExitPlanner_IsStepSafetyDanger(&g_activeProfile[g_activeStepIndex]) == TRUE)
    {
        AppAutoExitService_EnterBlocked();
        return;
    }

    if(AppAutoExitService_HasElapsed(g_stepStartTick,
                                     AppAutoExitService_GetCurrentStepDurationMs()) == FALSE)
    {
        return;
    }

    g_activeStepIndex++;

    if(g_activeStepIndex >= g_activeProfileCount)
    {
        AppAutoExitService_CompleteProfile();
        return;
    }

    g_stepStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(g_activeProfile[g_activeStepIndex].driveCmd,
                                  g_activeProfile[g_activeStepIndex].steeringCmd);
}

static void AppAutoExitService_StartAvoidEscape(void)
{
    g_avoidEscapeStartTick = xTaskGetTickCount();
    g_autoExitState = APP_AUTO_EXIT_STATE_AVOID_ESCAPE;

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_FORWARD,
                                  AppAutoExitPlanner_GetEscapeSteer(g_exitDirection));
}

static void AppAutoExitService_FinishAvoidEscape(void)
{
    g_avoidEscapeElapsedMs =
        AppAutoExitService_GetElapsedMs(g_avoidEscapeStartTick);

    g_autoExitState = APP_AUTO_EXIT_STATE_AVOID_STOP_1;
    g_stateStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_STOP,
                                  APP_AUTO_EXIT_STEER_CENTER);
}

static void AppAutoExitService_StartAvoidRealign(void)
{
    g_avoidRealignStartTick = xTaskGetTickCount();
    g_autoExitState = APP_AUTO_EXIT_STATE_AVOID_REALIGN;

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_FORWARD,
                                  AppAutoExitPlanner_GetRealignSteer(g_exitDirection));
}

static void AppAutoExitService_FinishAvoidRealign(void)
{
    g_autoExitState = APP_AUTO_EXIT_STATE_AVOID_STOP_2;
    g_stateStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_STOP,
                                  APP_AUTO_EXIT_STEER_CENTER);
}

static void AppAutoExitService_ApplyAvoidPlan(const AppAutoExitAvoidPlan *avoidPlan)
{
    if(avoidPlan == 0)
    {
        g_avoidEscapeMs = 0u;
        g_avoidRealignMs = 0u;
        return;
    }

    g_avoidEscapeMs = avoidPlan->escapeMs;
    g_avoidRealignMs = avoidPlan->realignMs;
}

static void AppAutoExitService_StartAutoExit(AppAutoExitDirection direction)
{
    if(g_autoExitState != APP_AUTO_EXIT_STATE_IDLE)
    {
        return;
    }

    AppAutoExitMonitor_Start(direction);

    g_exitDirection = direction;

    if(direction == APP_AUTO_EXIT_DIR_STRAIGHT)
    {
        AppAutoExitService_StartProfileForDirection(APP_AUTO_EXIT_DIR_STRAIGHT,
                                                    0u);
        return;
    }

    g_autoExitActive = TRUE;
    g_autoExitState = APP_AUTO_EXIT_STATE_START_STOP;
    g_stateStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_STOP,
                                  APP_AUTO_EXIT_STEER_CENTER);
}

static void AppAutoExitService_StartNormalExitProfile(void)
{
    AppAutoExitService_StartProfileForDirection(g_exitDirection, 0u);
}

static void AppAutoExitService_StartResumeExitProfile(void)
{
    uint32 firstStepReductionMs;

    firstStepReductionMs =
        AppAutoExitPlanner_CalcFirstStepReductionMs(g_avoidEscapeElapsedMs,
                                                    g_avoidRealignMs);

    AppAutoExitService_StartProfileForDirection(g_exitDirection,
                                                firstStepReductionMs);
}

static void AppAutoExitService_ServiceState(void)
{
    AppAutoExitStrategy strategy;
    AppAutoExitAvoidPlan avoidPlan;

    switch(g_autoExitState)
    {
        case APP_AUTO_EXIT_STATE_IDLE:
            break;

        case APP_AUTO_EXIT_STATE_START_STOP:
            if(AppAutoExitService_HasElapsed(g_stateStartTick,
                                             APP_AUTO_EXIT_START_STOP_MS) == TRUE)
            {
                g_autoExitState = APP_AUTO_EXIT_STATE_SELECT_STRATEGY;
            }
            break;

        case APP_AUTO_EXIT_STATE_SELECT_STRATEGY:
            strategy = AppAutoExitPlanner_SelectStrategy(g_exitDirection,
                                                         &avoidPlan);
            AppAutoExitService_ApplyAvoidPlan(&avoidPlan);

            if(strategy == APP_AUTO_EXIT_STRATEGY_NORMAL)
            {
                AppAutoExitService_StartNormalExitProfile();
            }
            else if(strategy == APP_AUTO_EXIT_STRATEGY_AVOID_AND_RESUME)
            {
                AppAutoExitService_StartAvoidEscape();
            }
            else
            {
                AppAutoExitService_EnterBlocked();
            }
            break;

        case APP_AUTO_EXIT_STATE_AVOID_ESCAPE:
            if(AppAutoExitPlanner_IsOppositeSideDangerDuringAvoid(g_exitDirection) == TRUE)
            {
                if(AppAutoExitService_HasElapsed(g_avoidEscapeStartTick,
                                                 APP_AUTO_EXIT_AVOID_ESCAPE_MIN_MS) == FALSE)
                {
                    AppAutoExitService_EnterBlocked();
                }
                else
                {
                    AppAutoExitService_FinishAvoidEscape();
                }
            }
            else if(AppAutoExitService_HasElapsed(g_avoidEscapeStartTick,
                                                  g_avoidEscapeMs) == TRUE)
            {
                AppAutoExitService_FinishAvoidEscape();
            }
            break;

        case APP_AUTO_EXIT_STATE_AVOID_STOP_1:
            if(AppAutoExitService_HasElapsed(g_stateStartTick,
                                             APP_AUTO_EXIT_SHIFT_STOP_MS) == TRUE)
            {
                AppAutoExitService_StartAvoidRealign();
            }
            break;

        case APP_AUTO_EXIT_STATE_AVOID_REALIGN:
            if(AppAutoExitService_HasElapsed(g_avoidRealignStartTick,
                                             g_avoidRealignMs) == TRUE)
            {
                AppAutoExitService_FinishAvoidRealign();
            }
            break;

        case APP_AUTO_EXIT_STATE_AVOID_STOP_2:
            if(AppAutoExitService_HasElapsed(g_stateStartTick,
                                             APP_AUTO_EXIT_SHIFT_STOP_MS) == TRUE)
            {
                AppAutoExitService_StartResumeExitProfile();
            }
            break;

        case APP_AUTO_EXIT_STATE_RUN_PROFILE:
            AppAutoExitService_ServiceProfile();
            break;

        case APP_AUTO_EXIT_STATE_BLOCKED:
        case APP_AUTO_EXIT_STATE_STOPPED:
            if(AppAutoExitService_HasElapsed(g_stateStartTick,
                                             APP_AUTO_EXIT_FINAL_STOP_MS) == TRUE)
            {
                AppAutoExitService_EnterIdle();
            }
            break;

        default:
            AppAutoExitService_EnterBlocked();
            break;
    }
}

static void AppAutoExitService_HandleCommand(AppAutoExitCmd command)
{
    switch(command)
    {
        case APP_AUTO_EXIT_CMD_START_STRAIGHT:
            AppAutoExitService_StartAutoExit(APP_AUTO_EXIT_DIR_STRAIGHT);
            break;

        case APP_AUTO_EXIT_CMD_START_LEFT:
            AppAutoExitService_StartAutoExit(APP_AUTO_EXIT_DIR_LEFT);
            break;

        case APP_AUTO_EXIT_CMD_START_RIGHT:
            AppAutoExitService_StartAutoExit(APP_AUTO_EXIT_DIR_RIGHT);
            break;

        case APP_AUTO_EXIT_CMD_STOP:
            if(g_autoExitState != APP_AUTO_EXIT_STATE_IDLE)
            {
                AppAutoExitService_EnterStopped();
            }
            else
            {
                AppAutoExitMonitor_SetIdle();
            }
            break;

        case APP_AUTO_EXIT_CMD_NORMAL:
            if(g_autoExitState == APP_AUTO_EXIT_STATE_IDLE)
            {
                AppAutoExitMonitor_SetIdle();
            }
            break;

        default:
            break;
    }
}

void AppAutoExitService_Init(void)
{
    AppAutoExitMonitor_Init();

    AppAutoExitService_EnterIdle();

    g_autoExitCmd.driveCmd = APP_AUTO_EXIT_DRIVE_STOP;
    g_autoExitCmd.steeringCmd = APP_AUTO_EXIT_STEER_CENTER;
}

boolean AppAutoExitService_IsActive(void)
{
    return g_autoExitActive;
}

BaseType_t AppAutoExitService_GetControlCommand(VehicleControlCmd_t *cmd)
{
    if(cmd == NULL)
    {
        return pdFAIL;
    }

    if((g_autoExitActive == FALSE) || (g_autoExitCmdValid == FALSE))
    {
        return pdFAIL;
    }

    *cmd = g_autoExitCmd;
    return pdPASS;
}

void AppAutoExitService_Task(void *arg)
{
    TickType_t lastWakeTime;
    AppAutoParkingState autoParking;
    static AppAutoExitCmd prevCmd = APP_AUTO_EXIT_CMD_NORMAL;

    (void)arg;

    lastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        if(AppRxService_GetAutoParkingState(&autoParking) == pdPASS)
        {
            if(autoParking.cmd != prevCmd)
            {
                AppAutoExitService_HandleCommand(autoParking.cmd);
                prevCmd = autoParking.cmd;
            }
        }

        AppAutoExitService_ServiceState();
        AppAutoExitMonitor_Service();

        vTaskDelayUntil(&lastWakeTime,
                        pdMS_TO_TICKS(APP_AUTO_EXIT_SERVICE_PERIOD_MS));
    }
}
