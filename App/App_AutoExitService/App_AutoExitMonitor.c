#include "App_AutoExitService_Internal.h"

#include "App_Can.h"
#include "App_RxService.h"
#include "task.h"

static AppAutoExitStatus g_exitStatus = APP_AUTO_EXIT_STATUS_IDLE;
static TickType_t g_exitResultStartTick = 0u;
static TickType_t g_lastStatusTxTick = 0u;

static sint16 g_exitStartYawDeg = 0;
static sint16 g_exitEndYawDeg = 0;
static sint16 g_exitYawDiffDeg = 0;
static boolean g_exitYawValid = FALSE;

static boolean AppAutoExitMonitor_HasElapsed(TickType_t startTick,
                                             uint32 durationMs)
{
    TickType_t nowTick;

    nowTick = xTaskGetTickCount();

    return ((nowTick - startTick) >= pdMS_TO_TICKS(durationMs)) ? TRUE : FALSE;
}

static boolean AppAutoExitMonitor_IsResultStatus(AppAutoExitStatus status)
{
    if((status == APP_AUTO_EXIT_STATUS_COMPLETE) ||
       (status == APP_AUTO_EXIT_STATUS_BLOCKED) ||
       (status == APP_AUTO_EXIT_STATUS_STOPPED))
    {
        return TRUE;
    }

    return FALSE;
}

static sint16 AppAutoExitMonitor_CalcYawDiffDeg(sint16 currentYawDeg,
                                                sint16 startYawDeg)
{
    sint32 diffDeg;

    diffDeg = (sint32)currentYawDeg - (sint32)startYawDeg;

    if(diffDeg > 180)
    {
        diffDeg -= 360;
    }
    else if(diffDeg < -180)
    {
        diffDeg += 360;
    }

    return (sint16)diffDeg;
}

#if (APP_AUTO_EXIT_YAW_VALIDATION_ENABLE != 0u)
static sint16 AppAutoExitMonitor_AbsSint16(sint16 value)
{
    return (value < 0) ? (sint16)(-value) : value;
}
#endif

static void AppAutoExitMonitor_ResetYaw(void)
{
    g_exitStartYawDeg = 0;
    g_exitEndYawDeg = 0;
    g_exitYawDiffDeg = 0;
    g_exitYawValid = FALSE;
}

static void AppAutoExitMonitor_CaptureStartYaw(void)
{
    AppUltrasonicState ultrasonic;

    if(AppRxService_GetUltrasonicState(&ultrasonic) == pdPASS)
    {
        g_exitStartYawDeg = ultrasonic.imuYaw;
        g_exitEndYawDeg = ultrasonic.imuYaw;
        g_exitYawDiffDeg = 0;
        g_exitYawValid = TRUE;
    }
    else
    {
        AppAutoExitMonitor_ResetYaw();
    }
}

static void AppAutoExitMonitor_CaptureEndYaw(void)
{
    AppUltrasonicState ultrasonic;

    if((g_exitYawValid == TRUE) &&
       (AppRxService_GetUltrasonicState(&ultrasonic) == pdPASS))
    {
        g_exitEndYawDeg = ultrasonic.imuYaw;
        g_exitYawDiffDeg =
            AppAutoExitMonitor_CalcYawDiffDeg(g_exitEndYawDeg,
                                              g_exitStartYawDeg);
    }
    else
    {
        g_exitYawValid = FALSE;
    }
}

static void AppAutoExitMonitor_UpdateCurrentYaw(void)
{
    if(g_exitYawValid == FALSE)
    {
        return;
    }

    AppAutoExitMonitor_CaptureEndYaw();
}

static boolean AppAutoExitMonitor_IsYawCompletionValid(AppAutoExitDirection direction)
{
#if (APP_AUTO_EXIT_YAW_VALIDATION_ENABLE == 0u)
    (void)direction;
    return TRUE;
#else
    sint16 absYawDiffDeg;

    if(g_exitYawValid == FALSE)
    {
        return FALSE;
    }

    absYawDiffDeg = AppAutoExitMonitor_AbsSint16(g_exitYawDiffDeg);

    if(direction == APP_AUTO_EXIT_DIR_STRAIGHT)
    {
        return (absYawDiffDeg <= APP_AUTO_EXIT_STRAIGHT_YAW_MAX_DEG) ? TRUE : FALSE;
    }

    if((absYawDiffDeg >= APP_AUTO_EXIT_TURN_YAW_MIN_DEG) &&
       (absYawDiffDeg <= APP_AUTO_EXIT_TURN_YAW_MAX_DEG))
    {
        return TRUE;
    }

    return FALSE;
#endif
}

static void AppAutoExitMonitor_ClearExpiredResult(void)
{
    if(AppAutoExitMonitor_IsResultStatus(g_exitStatus) == FALSE)
    {
        return;
    }

    if(AppAutoExitMonitor_HasElapsed(g_exitResultStartTick,
                                     APP_AUTO_EXIT_RESULT_HOLD_MS) == TRUE)
    {
        g_exitStatus = APP_AUTO_EXIT_STATUS_IDLE;
    }
}

static void AppAutoExitMonitor_SendStatus(AppAutoExitStatus status)
{
    ExitCompleteCmd_t tx;

    tx.autoparkingStatus = (uint8)status;

    (void)AppCan_SendExitComplete(&tx);
}

static void AppAutoExitMonitor_ServiceStatusTx(void)
{
    TickType_t nowTick;

    nowTick = xTaskGetTickCount();

    if((nowTick - g_lastStatusTxTick) >= pdMS_TO_TICKS(APP_AUTO_EXIT_STATUS_TX_PERIOD_MS))
    {
        AppAutoExitMonitor_SendStatus(g_exitStatus);
        g_lastStatusTxTick = nowTick;
    }
}

void AppAutoExitMonitor_Init(void)
{
    g_exitStatus = APP_AUTO_EXIT_STATUS_IDLE;
    g_exitResultStartTick = 0u;
    g_lastStatusTxTick = xTaskGetTickCount();

    AppAutoExitMonitor_ResetYaw();
}

void AppAutoExitMonitor_Start(void)
{
    g_exitStatus = APP_AUTO_EXIT_STATUS_IN_PROGRESS;
    AppAutoExitMonitor_CaptureStartYaw();
}

void AppAutoExitMonitor_SetIdle(void)
{
    g_exitStatus = APP_AUTO_EXIT_STATUS_IDLE;
}

void AppAutoExitMonitor_SetResult(AppAutoExitStatus status)
{
    g_exitStatus = status;
    g_exitResultStartTick = xTaskGetTickCount();
}

boolean AppAutoExitMonitor_FinishAndValidate(AppAutoExitDirection direction)
{
    AppAutoExitMonitor_CaptureEndYaw();

    return AppAutoExitMonitor_IsYawCompletionValid(direction);
}

void AppAutoExitMonitor_Service(void)
{
    if(g_exitStatus == APP_AUTO_EXIT_STATUS_IN_PROGRESS)
    {
        AppAutoExitMonitor_UpdateCurrentYaw();
    }

    AppAutoExitMonitor_ClearExpiredResult();
    AppAutoExitMonitor_ServiceStatusTx();
}
