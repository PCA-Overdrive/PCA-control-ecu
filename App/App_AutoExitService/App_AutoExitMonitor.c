#include "App_AutoExitService_Internal.h"

#include "App_Can.h"
#include "App_RxService.h"
#include "task.h"

#define APP_AUTO_EXIT_BASE_TURN_DEG        (90)
#define APP_AUTO_EXIT_YAW_TARGET_TOL_DEG   (15)

/*
 * IMU yaw에서 오른쪽 회전 시 yaw가 증가하면 +1
 * 오른쪽 회전 시 yaw가 감소하면 -1
 * 실제 IMU 들어오면 테스트해서 결정.
 */
#define APP_AUTO_EXIT_IMU_RIGHT_SIGN       (1)

static AppAutoExitStatus g_exitStatus = APP_AUTO_EXIT_STATUS_IDLE;
static TickType_t g_exitResultStartTick = 0u;
static TickType_t g_lastStatusTxTick = 0u;

static sint16 g_exitStartYawDeg = 0;
static sint16 g_exitEndYawDeg = 0;
static sint16 g_exitYawDiffDeg = 0;
static boolean g_exitYawValid = FALSE;

static sint16 g_exitLineAngleDeg = 0;
static sint16 g_exitTargetTurnDeg = 0;
static sint16 g_exitTargetYawDeg = 0;
static sint16 g_exitYawErrorDeg = 0;

/* 디버거 확인용 */
volatile sint16 g_dbg_autoExitLineAngleDeg = 0;
volatile sint16 g_dbg_autoExitTargetTurnDeg = 0;
volatile sint16 g_dbg_autoExitTargetYawDeg = 0;
volatile sint16 g_dbg_autoExitYawErrorDeg = 0;

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

static sint16 AppAutoExitMonitor_NormalizeYawDeg(sint16 yawDeg)
{
    while(yawDeg > 180)
    {
        yawDeg -= 360;
    }

    while(yawDeg < -180)
    {
        yawDeg += 360;
    }

    return yawDeg;
}

static sint16 AppAutoExitMonitor_CalcYawDiffDeg(sint16 currentYawDeg,
                                                sint16 startYawDeg)
{
    return AppAutoExitMonitor_NormalizeYawDeg(currentYawDeg - startYawDeg);
}

static sint16 AppAutoExitMonitor_CalcYawErrorToTargetDeg(sint16 targetYawDeg,
                                                         sint16 currentYawDeg)
{
    return AppAutoExitMonitor_NormalizeYawDeg(targetYawDeg - currentYawDeg);
}

#if (APP_AUTO_EXIT_YAW_VALIDATION_ENABLE != 0u)
static sint16 AppAutoExitMonitor_AbsSint16(sint16 value)
{
    return (value < 0) ? (sint16)(-value) : value;
}
#endif

static sint16 AppAutoExitMonitor_CalcTargetTurnDeg(AppAutoExitDirection direction,
                                                   sint16 lineAngleDeg)
{
    sint16 targetTurnDeg;

    if(direction == APP_AUTO_EXIT_DIR_STRAIGHT)
    {
        return 0;
    }

    if(direction == APP_AUTO_EXIT_DIR_RIGHT)
    {
        /*
         * lineAngleDeg > 0: 왼쪽으로 기울어짐
         * 오른쪽 출차는 더 많이 돌아야 함.
         */
        targetTurnDeg = APP_AUTO_EXIT_BASE_TURN_DEG + lineAngleDeg;
    }
    else
    {
        /*
         * 왼쪽 출차는 반대.
         * 왼쪽으로 이미 2도 기울어져 있으면 88도만 돌면 됨.
         */
        targetTurnDeg = APP_AUTO_EXIT_BASE_TURN_DEG - lineAngleDeg;
    }

    if(targetTurnDeg < 45)
    {
        targetTurnDeg = 45;
    }
    else if(targetTurnDeg > 135)
    {
        targetTurnDeg = 135;
    }

    return targetTurnDeg;
}

static sint16 AppAutoExitMonitor_CalcTargetYawDeg(sint16 startYawDeg,
                                                  AppAutoExitDirection direction,
                                                  sint16 targetTurnDeg)
{
    sint16 turnSign;

    if(direction == APP_AUTO_EXIT_DIR_RIGHT)
    {
        turnSign = APP_AUTO_EXIT_IMU_RIGHT_SIGN;
    }
    else if(direction == APP_AUTO_EXIT_DIR_LEFT)
    {
        turnSign = (sint16)(-APP_AUTO_EXIT_IMU_RIGHT_SIGN);
    }
    else
    {
        turnSign = 0;
    }

    return AppAutoExitMonitor_NormalizeYawDeg(startYawDeg +
                                              (sint16)(turnSign * targetTurnDeg));
}

static void AppAutoExitMonitor_ResetYaw(void)
{
    g_exitStartYawDeg = 0;
    g_exitEndYawDeg = 0;
    g_exitYawDiffDeg = 0;
    g_exitYawValid = FALSE;
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

        g_exitYawErrorDeg =
            AppAutoExitMonitor_CalcYawErrorToTargetDeg(g_exitTargetYawDeg,
                                                       g_exitEndYawDeg);

        g_dbg_autoExitYawErrorDeg = g_exitYawErrorDeg;
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
    sint16 absYawErrorDeg;

    (void)direction;

    if(g_exitYawValid == FALSE)
    {
        return FALSE;
    }

    absYawErrorDeg = AppAutoExitMonitor_AbsSint16(g_exitYawErrorDeg);

    return (absYawErrorDeg <= APP_AUTO_EXIT_YAW_TARGET_TOL_DEG) ? TRUE : FALSE;
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

void AppAutoExitMonitor_Start(AppAutoExitDirection direction)
{
    AppUltrasonicState ultrasonic;
    AppRpiInputState rpiInput;

    g_exitStatus = APP_AUTO_EXIT_STATUS_IN_PROGRESS;

    AppAutoExitMonitor_ResetYaw();

    g_exitLineAngleDeg = 0;
    g_exitTargetTurnDeg = 0;
    g_exitTargetYawDeg = 0;
    g_exitYawErrorDeg = 0;

    if(AppRxService_GetRpiInput(&rpiInput) == pdPASS)
    {
        g_exitLineAngleDeg = rpiInput.lineAngleDeg;
    }

    if(AppRxService_GetUltrasonicState(&ultrasonic) == pdPASS)
    {
        g_exitStartYawDeg = ultrasonic.imuYaw;
        g_exitEndYawDeg = ultrasonic.imuYaw;
        g_exitYawDiffDeg = 0;
        g_exitYawValid = TRUE;
    }

    g_exitTargetTurnDeg =
        AppAutoExitMonitor_CalcTargetTurnDeg(direction, g_exitLineAngleDeg);

    if(g_exitYawValid == TRUE)
    {
        g_exitTargetYawDeg =
            AppAutoExitMonitor_CalcTargetYawDeg(g_exitStartYawDeg,
                                                direction,
                                                g_exitTargetTurnDeg);

        g_exitYawErrorDeg =
            AppAutoExitMonitor_CalcYawErrorToTargetDeg(g_exitTargetYawDeg,
                                                       g_exitEndYawDeg);
    }

    g_dbg_autoExitLineAngleDeg = g_exitLineAngleDeg;
    g_dbg_autoExitTargetTurnDeg = g_exitTargetTurnDeg;
    g_dbg_autoExitTargetYawDeg = g_exitTargetYawDeg;
    g_dbg_autoExitYawErrorDeg = g_exitYawErrorDeg;
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
