#include "App_AutoExitService_Internal.h"

#include "App_Can.h"
#include "App_RxService.h"
#include "task.h"

/*
 * 자동출차 모니터링 컨텍스트
 *
 * 이 파일은 자동출차의 실제 주행 시퀀스를 만드는 파일이 아니라,
 * 자동출차 상태를 관리하고, 완료/정지 상태를 0x401로 주기 송신하며,
 * 필요하면 IMU yaw를 이용해 출차 완료 각도가 맞는지 검증하는 역할을 한다.
 */
typedef struct
{
    /* 현재 자동출차 상태
     * IDLE / IN_PROGRESS / COMPLETE / STOPPED 등
     */
    AppAutoExitStatus status;

    /* COMPLETE / STOPPED / BLOCKED 상태가 된 시점
     * 결과 상태를 일정 시간 유지한 뒤 IDLE로 되돌리기 위해 사용
     */
    TickType_t resultStartTick;

    /* 0x401 상태 메시지를 마지막으로 보낸 tick
     * 주기 송신 간격 계산용
     */
    TickType_t lastStatusTxTick;

    /* 자동출차 시작 시점의 yaw */
    sint16 startYawDeg;

    /* 자동출차 종료 또는 진행 중 현재 yaw */
    sint16 endYawDeg;

    /* RPi에서 받은 주차선/차량 기준 각도 보정값 */
    sint16 lineAngleDeg;

    /* 목표 회전 각도
     * 기본 회전각 + lineAngle 보정을 반영한 값
     */
    sint16 targetTurnDeg;

    /* 목표 yaw
     * startYawDeg에서 targetTurnDeg만큼 회전한 최종 목표 방향
     */
    sint16 targetYawDeg;

    /* 목표 yaw와 현재 yaw의 오차 */
    sint16 yawErrorDeg;

    /* yaw 검증에 사용할 수 있는 유효한 IMU 값이 있는지 여부 */
    boolean yawValid;
} AppAutoExitMonitorContext;

/* 자동출차 모니터링 전역 상태 */
static AppAutoExitMonitorContext g_monitor;

/*
 * yaw 각도를 -180 ~ +180 범위로 정규화
 *
 * 예:
 *  190  -> -170
 * -190  ->  170
 */
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

/*
 * 출차 방향과 lineAngle을 이용해 목표 회전 각도를 계산
 *
 * 직진 출차:
 *  - 회전 목표 없음
 *
 * 우측 출차:
 *  - 기본 회전각 + lineAngle
 *
 * 좌측 출차:
 *  - 기본 회전각 - lineAngle
 *
 * 계산된 회전각은 최소/최대 제한값 안으로 clamp한다.
 */
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
        targetTurnDeg = (sint16)(APP_AUTO_EXIT_BASE_TURN_DEG + lineAngleDeg);
    }
    else
    {
        targetTurnDeg = (sint16)(APP_AUTO_EXIT_BASE_TURN_DEG - lineAngleDeg);
    }

    if(targetTurnDeg < APP_AUTO_EXIT_TARGET_TURN_MIN_DEG)
    {
        targetTurnDeg = APP_AUTO_EXIT_TARGET_TURN_MIN_DEG;
    }
    else if(targetTurnDeg > APP_AUTO_EXIT_TARGET_TURN_MAX_DEG)
    {
        targetTurnDeg = APP_AUTO_EXIT_TARGET_TURN_MAX_DEG;
    }

    return targetTurnDeg;
}

/*
 * 시작 yaw, 출차 방향, 목표 회전각을 이용해 최종 목표 yaw 계산
 *
 * APP_AUTO_EXIT_IMU_RIGHT_SIGN은 IMU yaw에서
 * 우회전이 +방향인지 -방향인지 보정하기 위한 값이다.
 */
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

    return AppAutoExitMonitor_NormalizeYawDeg(
        (sint16)(startYawDeg + (sint16)(turnSign * targetTurnDeg)));
}

/*
 * yaw 관련 모니터링 값 초기화
 *
 * 자동출차 시작 전 또는 초기화 시 호출된다.
 */
static void AppAutoExitMonitor_ResetYaw(void)
{
    g_monitor.startYawDeg = 0;
    g_monitor.endYawDeg = 0;
    g_monitor.lineAngleDeg = 0;
    g_monitor.targetTurnDeg = 0;
    g_monitor.targetYawDeg = 0;
    g_monitor.yawErrorDeg = 0;
    g_monitor.yawValid = FALSE;
}

/*
 * 현재 yaw를 읽어서 endYawDeg와 yawErrorDeg를 갱신
 *
 * 자동출차 진행 중에는 계속 현재 yaw를 갱신하고,
 * 완료 시점에는 마지막 yaw를 기준으로 목표 yaw와의 오차를 계산한다.
 */
static void AppAutoExitMonitor_CaptureEndYaw(void)
{
    AppUltrasonicState ultrasonic;

    if((g_monitor.yawValid == TRUE) &&
       (AppRxService_GetUltrasonicState(&ultrasonic) == pdPASS))
    {
        g_monitor.endYawDeg = ultrasonic.imuYaw;

        g_monitor.yawErrorDeg =
            AppAutoExitMonitor_NormalizeYawDeg(
                (sint16)(g_monitor.targetYawDeg - g_monitor.endYawDeg));
    }
    else
    {
        /*
         * IMU 값을 읽지 못하면 yaw 기반 완료 검증은 불가능하므로 invalid 처리
         */
        g_monitor.yawValid = FALSE;
    }
}

/*
 * AutoExitMonitor 초기화
 *
 * 초기 상태는 IDLE이며, yaw 관련 값도 초기화한다.
 */
void AppAutoExitMonitor_Init(void)
{
    g_monitor.status = APP_AUTO_EXIT_STATUS_IDLE;
    g_monitor.resultStartTick = 0u;
    g_monitor.lastStatusTxTick = xTaskGetTickCount();

    AppAutoExitMonitor_ResetYaw();
}

/*
 * 자동출차 시작 시 호출
 *
 * 역할:
 *  1. 상태를 IN_PROGRESS로 변경
 *  2. RPi 입력에서 lineAngleDeg 읽기
 *  3. 초음파 상태에 포함된 IMU yaw 읽기
 *  4. 목표 회전각 targetTurnDeg 계산
 *  5. 목표 yaw targetYawDeg 계산
 */
void AppAutoExitMonitor_Start(AppAutoExitDirection direction)
{
    AppUltrasonicState ultrasonic;
    AppRpiInputState rpiInput;

    g_monitor.status = APP_AUTO_EXIT_STATUS_IN_PROGRESS;
    AppAutoExitMonitor_ResetYaw();

    /*
     * RPi 입력에서 주차선 또는 출차 기준선 각도 보정값을 가져온다.
     * 수신값이 없으면 lineAngleDeg는 ResetYaw()에서 초기화된 0을 사용한다.
     */
    if(AppRxService_GetRpiInput(&rpiInput) == pdPASS)
    {
        g_monitor.lineAngleDeg = rpiInput.lineAngleDeg;
    }

    /*
     * 자동출차 시작 시점의 IMU yaw를 저장한다.
     * 이 값이 있어야 목표 yaw와 최종 yaw 오차를 계산할 수 있다.
     */
    if(AppRxService_GetUltrasonicState(&ultrasonic) == pdPASS)
    {
        g_monitor.startYawDeg = ultrasonic.imuYaw;
        g_monitor.endYawDeg = ultrasonic.imuYaw;
        g_monitor.yawValid = TRUE;
    }

    /*
     * 출차 방향과 lineAngleDeg를 반영해 목표 회전 각도 계산
     */
    g_monitor.targetTurnDeg =
        AppAutoExitMonitor_CalcTargetTurnDeg(direction,
                                             g_monitor.lineAngleDeg);

    /*
     * 시작 yaw가 유효하면 최종 목표 yaw와 현재 yaw 오차 계산
     */
    if(g_monitor.yawValid == TRUE)
    {
        g_monitor.targetYawDeg =
            AppAutoExitMonitor_CalcTargetYawDeg(g_monitor.startYawDeg,
                                                direction,
                                                g_monitor.targetTurnDeg);

        g_monitor.yawErrorDeg = AppAutoExitMonitor_NormalizeYawDeg((sint16)(g_monitor.targetYawDeg - g_monitor.endYawDeg));
    }
}

/*
 * 자동출차 상태를 IDLE로 변경
 *
 * 외부에서 자동출차를 완전히 대기 상태로 돌릴 때 사용한다.
 */
void AppAutoExitMonitor_SetIdle(void)
{
    g_monitor.status = APP_AUTO_EXIT_STATUS_IDLE;
    g_monitor.resultStartTick = 0u;
}

/*
 * 자동출차 결과 상태 설정
 *
 * COMPLETE / STOPPED / BLOCKED 같은 결과 상태를 설정하고,
 * 그 상태를 일정 시간 유지하기 위해 시작 tick을 저장한다.
 */
void AppAutoExitMonitor_SetResult(AppAutoExitStatus status)
{
    g_monitor.status = status;
    g_monitor.resultStartTick = xTaskGetTickCount();
}

/*
 * 자동출차 종료 시 yaw 검증 수행
 *
 * 현재 yaw를 한 번 더 캡처한 뒤,
 * 목표 yaw와의 오차가 허용 범위 안인지 확인한다.
 */
boolean AppAutoExitMonitor_FinishAndValidate(void)
{
#if (APP_AUTO_EXIT_YAW_VALIDATION_ENABLE != 0u)
    sint16 absYawErrorDeg;
#endif

    AppAutoExitMonitor_CaptureEndYaw();

#if (APP_AUTO_EXIT_YAW_VALIDATION_ENABLE == 0u)
    return TRUE;
#else
    if(g_monitor.yawValid == FALSE)
    {
        return FALSE;
    }

    absYawErrorDeg = g_monitor.yawErrorDeg;

    if(absYawErrorDeg < 0)
    {
        absYawErrorDeg = (sint16)(-absYawErrorDeg);
    }

    return (absYawErrorDeg <= APP_AUTO_EXIT_YAW_TARGET_TOL_DEG) ? TRUE : FALSE;
#endif
}

/*
 * AutoExitMonitor 주기 서비스 함수
 *
 * AppAutoExitService 쪽에서 주기적으로 호출된다.
 *
 * 역할:
 *  1. IN_PROGRESS 상태이면 현재 yaw 계속 갱신
 *  2. COMPLETE / STOPPED / BLOCKED 결과 상태가 오래 유지되면 IDLE로 복귀
 *  3. 0x401 상태 메시지 주기 송신
 */
void AppAutoExitMonitor_Service(void)
{
    TickType_t nowTick;
    ExitCompleteCmd_t tx;

    nowTick = xTaskGetTickCount();

    /*
     * 자동출차 진행 중이면 현재 yaw를 계속 갱신한다.
     */
    if(g_monitor.status == APP_AUTO_EXIT_STATUS_IN_PROGRESS)
    {
        AppAutoExitMonitor_CaptureEndYaw();
    }

    /*
     * 결과 상태를 일정 시간 유지한 뒤 IDLE로 되돌린다.
     *
     * 주의:
     * 기존 코드에서는 COMPLETE / STOPPED만 결과 상태로 봤다.
     * 그런데 EnterBlocked()에서도 APP_AUTO_EXIT_STATUS_BLOCKED를 설정하므로,
     * BLOCKED도 일정 시간 뒤 IDLE로 되돌리는 것이 자연스럽다.
     */
    if((g_monitor.status == APP_AUTO_EXIT_STATUS_COMPLETE) ||
       (g_monitor.status == APP_AUTO_EXIT_STATUS_STOPPED) ||
       (g_monitor.status == APP_AUTO_EXIT_STATUS_BLOCKED))
    {
        if((nowTick - g_monitor.resultStartTick) >=
           pdMS_TO_TICKS(APP_AUTO_EXIT_RESULT_HOLD_MS))
        {
            g_monitor.status = APP_AUTO_EXIT_STATUS_IDLE;
        }
    }

    /*
     * 0x401 상태 메시지 주기 송신
     */
    if((nowTick - g_monitor.lastStatusTxTick) >=
       pdMS_TO_TICKS(APP_AUTO_EXIT_STATUS_TX_PERIOD_MS))
    {
        tx.autoparkingStatus = (uint8)g_monitor.status;

        (void)AppCan_SendExitComplete(&tx);

        g_monitor.lastStatusTxTick = nowTick;
    }
}
