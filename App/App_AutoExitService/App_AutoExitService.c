#include "App_AutoExitService.h"
#include "App_AutoExitService_Internal.h"

#include "App_RxService.h"
#include "task.h"

/*
 * 자동출차 서비스 내부 상태
 *
 * 이 enum은 자동출차가 현재 어떤 단계에 있는지 나타낸다.
 * AppAutoExitService_ServiceState()에서 이 상태값을 기준으로 상태머신이 진행된다.
 */
typedef enum
{
    /* 대기 상태: 자동출차 동작 없음 */
    APP_AUTO_EXIT_STATE_IDLE = 0,

    /* 자동출차 시작 직후 잠깐 정지하는 상태 */
    APP_AUTO_EXIT_STATE_START_STOP,

    /* NORMAL / AVOID / BLOCKED 전략을 선택하는 상태 */
    APP_AUTO_EXIT_STATE_SELECT_STRATEGY,

    /* 회피 동작 1단계: 출차 반대 방향으로 살짝 빠지는 상태 */
    APP_AUTO_EXIT_STATE_AVOID_ESCAPE,

    /* 회피 escape 후 잠깐 정지하는 상태 */
    APP_AUTO_EXIT_STATE_AVOID_STOP_1,

    /* 회피 동작 2단계: 다시 원래 출차 방향으로 정렬하는 상태 */
    APP_AUTO_EXIT_STATE_AVOID_REALIGN,

    /* realign 후 잠깐 정지하는 상태 */
    APP_AUTO_EXIT_STATE_AVOID_STOP_2,

    /* 실제 자동출차 motion profile을 순서대로 실행하는 상태 */
    APP_AUTO_EXIT_STATE_RUN_PROFILE,

    /* 장애물 등으로 출차가 불가능하다고 판단된 상태 */
    APP_AUTO_EXIT_STATE_BLOCKED,

    /* 외부 STOP 명령으로 자동출차가 중단된 상태 */
    APP_AUTO_EXIT_STATE_STOPPED
} AppAutoExitState;

/*
 * 자동출차 profile 실행 상태
 *
 * profile은 여러 개의 motion step 배열이다.
 * 예:
 *  1. 전진
 *  2. 조향하며 전진
 *  3. 정지
 *  4. 후진
 *
 * 이 구조체는 현재 몇 번째 step을 실행 중인지,
 * 해당 step을 언제 시작했는지 저장한다.
 */
typedef struct
{
    /* 실행할 motion step 배열 포인터 */
    const AppAutoExitMotionStep *steps;

    /* 전체 step 개수 */
    uint32 count;

    /* 현재 실행 중인 step index */
    uint32 index;

    /* 현재 step 시작 tick */
    TickType_t startTick;

    /*
     * 회피 동작을 먼저 수행했을 경우,
     * 기존 첫 번째 전진 시간이 너무 길어질 수 있으므로
     * 첫 번째 step 시간을 줄이기 위한 값
     */
    uint32 firstStepReductionMs;
} AppAutoExitProfileRuntime;

typedef struct
{
    /*
     * Planner가 계산한 회피 계획
     *
     * plan 안에는:
     *  - escapeMs
     *  - realignMs
     *  - escapeSteerCmd
     *  - realignSteerCmd
     * 가 들어간다.
     */
    AppAutoExitAvoidPlan plan;

    /* escape 시작 tick */
    TickType_t escapeStartTick;

    /* realign 시작 tick */
    TickType_t realignStartTick;

    /*
     * 실제 escape에 걸린 시간
     * 중간에 반대편 위험이 감지되어 escape가 조기 종료될 수 있으므로 따로 저장한다.
     */
    uint32 escapeElapsedMs;
} AppAutoExitAvoidRuntime;
/*
 * 자동출차 서비스 전체 context
 *
 * 이 구조체 하나가 자동출차 서비스의 현재 상태를 모두 들고 있다.
 */
typedef struct
{
    /*
     * 자동출차 제어가 활성화되어 있는지 여부
     *
     * TRUE이면 DriveService가 AppAutoExitService_GetControlCommand()를 통해
     * 자동출차 명령을 가져가 0x100 제어 명령에 반영할 수 있다.
     */
    boolean active;

    /*
     * 현재 g_autoExit.cmd가 유효한지 여부
     *
     * active가 TRUE여도 cmdValid가 FALSE이면
     * 아직 내보낼 제어 명령이 없다는 뜻이다.
     */
    boolean cmdValid;

    /* Motor ECU로 보낼 drive/steering 제어 명령 */
    VehicleControlCmd_t cmd;

    /*
     * 마지막으로 처리한 자동출차 명령
     *
     * 같은 명령이 계속 주기 송신될 때
     * 매번 StartAutoExit()가 다시 호출되는 것을 막기 위해 사용한다.
     */
    AppAutoExitCmd lastCommand;

    /* 현재 자동출차 상태머신 상태 */
    AppAutoExitState state;

    /* 현재 자동출차 방향: 직진 / 좌측 / 우측 */
    AppAutoExitDirection direction;

    /* 현재 state에 진입한 tick */
    TickType_t stateStartTick;

    /* motion profile 실행 정보 */
    AppAutoExitProfileRuntime profile;

    /* 회피 동작 실행 정보 */
    AppAutoExitAvoidRuntime avoid;
} AppAutoExitServiceContext;

/* 자동출차 서비스 전역 context */
static AppAutoExitServiceContext g_autoExit;

/*
 * profile 실행 상태 초기화
 *
 * 자동출차가 끝나거나 IDLE로 돌아갈 때 호출된다.
 */
static void AppAutoExitService_ResetProfile(void)
{
    g_autoExit.profile.steps = 0;
    g_autoExit.profile.count = 0u;
    g_autoExit.profile.index = 0u;
    g_autoExit.profile.startTick = 0u;
    g_autoExit.profile.firstStepReductionMs = 0u;
}

/*
 * 회피 계획 및 실행 상태 초기화
 */
static void AppAutoExitService_ResetAvoidPlan(void)
{
    g_autoExit.avoid.plan.escapeMs = 0u;
    g_autoExit.avoid.plan.realignMs = 0u;
    g_autoExit.avoid.plan.escapeSteerCmd = APP_AUTO_EXIT_STEER_CENTER;
    g_autoExit.avoid.plan.realignSteerCmd = APP_AUTO_EXIT_STEER_CENTER;

    g_autoExit.avoid.escapeStartTick = 0u;
    g_autoExit.avoid.realignStartTick = 0u;
    g_autoExit.avoid.escapeElapsedMs = 0u;
}

/*
 * startTick 이후 durationMs가 지났는지 확인
 *
 * 상태머신에서 특정 상태를 일정 시간 유지할 때 사용한다.
 */
static boolean AppAutoExitService_HasElapsed(TickType_t startTick,
                                             uint32 durationMs)
{
    TickType_t nowTick;

    nowTick = xTaskGetTickCount();

    return ((nowTick - startTick) >= pdMS_TO_TICKS(durationMs)) ? TRUE : FALSE;
}

/*
 * startTick 이후 실제로 몇 ms가 지났는지 계산
 *
 * 회피 escape가 조기 종료되었을 때,
 * 실제 escape 수행 시간을 저장하기 위해 사용한다.
 */
static uint32 AppAutoExitService_GetElapsedMs(TickType_t startTick)
{
    TickType_t elapsedTick;

    elapsedTick = xTaskGetTickCount() - startTick;

    return (uint32)((elapsedTick * 1000u) / configTICK_RATE_HZ);
}

/*
 * 현재 자동출차 제어 명령 설정
 *
 * DriveService는 이 cmd를 가져가서 Motor ECU 제어 명령으로 사용한다.
 */
static void AppAutoExitService_SetCommand(uint8 driveCmd, uint8 steeringCmd)
{
    g_autoExit.cmd.driveCmd = driveCmd;
    g_autoExit.cmd.steeringCmd = steeringCmd;
    g_autoExit.cmdValid = TRUE;
}

/*
 * 자동출차 서비스를 IDLE 상태로 전환
 *
 * active와 cmdValid를 FALSE로 내려서,
 * DriveService가 더 이상 자동출차 명령을 사용하지 않게 한다.
 */
static void AppAutoExitService_EnterIdle(void)
{
    g_autoExit.active = FALSE;
    g_autoExit.cmdValid = FALSE;
    g_autoExit.state = APP_AUTO_EXIT_STATE_IDLE;
    g_autoExit.direction = APP_AUTO_EXIT_DIR_STRAIGHT;

    AppAutoExitService_ResetProfile();
    AppAutoExitService_ResetAvoidPlan();
}

/*
 * 일정 시간 정지 상태로 들어가는 공통 함수
 *
 * BLOCKED, STOPPED 같은 상태는
 * 바로 IDLE로 가지 않고 일정 시간 정지 명령을 유지한다.
 */
static void AppAutoExitService_EnterTimedStopState(AppAutoExitState state)
{
    g_autoExit.active = TRUE;
    g_autoExit.state = state;
    g_autoExit.stateStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_STOP,
                                  APP_AUTO_EXIT_STEER_CENTER);
}

/*
 * 출차 불가 상태 진입
 *
 * Monitor에는 BLOCKED 결과를 기록하고,
 * 차량 제어는 정지 명령으로 유지한다.
 */
static void AppAutoExitService_EnterBlocked(void)
{
    AppAutoExitMonitor_SetResult(APP_AUTO_EXIT_STATUS_BLOCKED);
    AppAutoExitService_EnterTimedStopState(APP_AUTO_EXIT_STATE_BLOCKED);
}

/*
 * 사용자 STOP 명령에 의해 자동출차가 중단된 상태 진입
 */
static void AppAutoExitService_EnterStopped(void)
{
    AppAutoExitMonitor_SetResult(APP_AUTO_EXIT_STATUS_STOPPED);
    AppAutoExitService_EnterTimedStopState(APP_AUTO_EXIT_STATE_STOPPED);
}

/*
 * profile 실행 종료
 *
 * 내부 상태는 IDLE로 돌리되,
 * 마지막 제어 명령은 정지/중앙 조향으로 남겨둔다.
 */
static void AppAutoExitService_StopProfile(void)
{
    AppAutoExitService_EnterIdle();

    g_autoExit.cmd.driveCmd = APP_AUTO_EXIT_DRIVE_STOP;
    g_autoExit.cmd.steeringCmd = APP_AUTO_EXIT_STEER_CENTER;
}

/*
 * 출차 방향에 맞는 profile을 가져와 실행 시작
 */
static void AppAutoExitService_StartProfileForDirection(AppAutoExitDirection direction,
                                                        uint32 firstStepReductionMs)
{
    const AppAutoExitMotionStep *profile;
    uint32 profileCount;

    profile = AppAutoExitProfile_Get(direction, &profileCount);

    /*
     * profile이 없거나 step 개수가 0이면
     * 실행할 수 없으므로 BLOCKED 처리
     */
    if((profile == 0) || (profileCount == 0u))
    {
        AppAutoExitService_EnterBlocked();
        return;
    }

    g_autoExit.profile.steps = profile;
    g_autoExit.profile.count = profileCount;
    g_autoExit.profile.index = 0u;
    g_autoExit.profile.firstStepReductionMs = firstStepReductionMs;
    g_autoExit.profile.startTick = xTaskGetTickCount();

    g_autoExit.active = TRUE;
    g_autoExit.state = APP_AUTO_EXIT_STATE_RUN_PROFILE;

    /*
     * 첫 번째 step의 제어 명령을 바로 적용
     */
    AppAutoExitService_SetCommand(g_autoExit.profile.steps[0].driveCmd,
                                  g_autoExit.profile.steps[0].steeringCmd);
}

/*
 * 현재 실행 중인 step의 duration을 계산
 *
 * 회피 동작을 먼저 수행한 경우,
 * 첫 번째 step은 firstStepReductionMs만큼 줄여서 실행한다.
 */
static uint32 AppAutoExitService_GetCurrentStepDurationMs(void)
{
    uint32 durationMs;

    durationMs = g_autoExit.profile.steps[g_autoExit.profile.index].durationMs;

    if((g_autoExit.profile.index == 0u) &&
       (g_autoExit.profile.firstStepReductionMs > 0u))
    {
        /*
         * 감소량이 원래 duration보다 크면
         * 최소 step 시간만 남긴다.
         */
        if(g_autoExit.profile.firstStepReductionMs >= durationMs)
        {
            durationMs = APP_AUTO_EXIT_MIN_STEP_MS;
        }
        else
        {
            durationMs = durationMs - g_autoExit.profile.firstStepReductionMs;
        }
    }

    return durationMs;
}

/*
 * profile 전체 완료 처리
 *
 * Monitor에서 yaw 검증까지 수행한 뒤,
 * 성공이면 COMPLETE, 실패이면 BLOCKED로 기록한다.
 */
static void AppAutoExitService_CompleteProfile(void)
{
    if(AppAutoExitMonitor_FinishAndValidate() == TRUE)
    {
        AppAutoExitMonitor_SetResult(APP_AUTO_EXIT_STATUS_COMPLETE);
    }
    else
    {
        AppAutoExitMonitor_SetResult(APP_AUTO_EXIT_STATUS_BLOCKED);
    }

    AppAutoExitService_StopProfile();
}

/*
 * RUN_PROFILE 상태 처리
 *
 * 현재 step을 계속 수행하다가 시간이 지나면 다음 step으로 넘어간다.
 * 진행 중 PDW DANGER가 감지되면 BLOCKED 처리한다.
 */
static void AppAutoExitService_ServiceProfile(void)
{
    if(g_autoExit.state != APP_AUTO_EXIT_STATE_RUN_PROFILE)
    {
        return;
    }

    /*
     * profile이 비정상이면 중단
     */
    if((g_autoExit.profile.steps == 0) ||
       (g_autoExit.profile.index >= g_autoExit.profile.count))
    {
        AppAutoExitService_StopProfile();
        return;
    }

    /*
     * 현재 step 방향 기준으로 DANGER가 있으면 출차 불가 처리
     */
    if(AppAutoExitPlanner_IsStepSafetyDanger(
           &g_autoExit.profile.steps[g_autoExit.profile.index]) == TRUE)
    {
        AppAutoExitService_EnterBlocked();
        return;
    }

    /*
     * 현재 step duration이 아직 끝나지 않았으면 계속 같은 명령 유지
     */
    if(AppAutoExitService_HasElapsed(g_autoExit.profile.startTick,
                                     AppAutoExitService_GetCurrentStepDurationMs()) == FALSE)
    {
        return;
    }

    /*
     * 현재 step 완료 → 다음 step으로 이동
     */
    g_autoExit.profile.index++;

    /*
     * 모든 step이 끝났으면 profile 완료 처리
     */
    if(g_autoExit.profile.index >= g_autoExit.profile.count)
    {
        AppAutoExitService_CompleteProfile();
        return;
    }

    /*
     * 다음 step 시작
     */
    g_autoExit.profile.startTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(
        g_autoExit.profile.steps[g_autoExit.profile.index].driveCmd,
        g_autoExit.profile.steps[g_autoExit.profile.index].steeringCmd);
}

/*
 * 회피 escape 시작
 *
 * 출차 방향의 반대 방향으로 조향하면서 전진한다.
 */
static void AppAutoExitService_StartAvoidEscape(void)
{
    g_autoExit.avoid.escapeStartTick = xTaskGetTickCount();
    g_autoExit.state = APP_AUTO_EXIT_STATE_AVOID_ESCAPE;

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_FORWARD,
                                  g_autoExit.avoid.plan.escapeSteerCmd);
}

static void AppAutoExitService_AdjustRealignTimeByEscapeElapsed(void)
{
    uint32 plannedEscapeMs;
    uint32 plannedRealignMs;
    uint32 actualEscapeMs;
    uint32 scaledRealignMs;

    plannedEscapeMs = g_autoExit.avoid.plan.escapeMs;
    plannedRealignMs = g_autoExit.avoid.plan.realignMs;
    actualEscapeMs = g_autoExit.avoid.escapeElapsedMs;

    /*
     * 회피 계획이 없거나 값이 비정상인 경우 방어
     */
    if((plannedEscapeMs == 0u) || (plannedRealignMs == 0u))
    {
        return;
    }

    /*
     * escape를 계획 시간만큼 수행했거나,
     * task tick 오차로 계획 시간보다 길게 수행했다면
     * realign은 기존 계획 시간을 그대로 사용한다.
     */
    if(actualEscapeMs >= plannedEscapeMs)
    {
        return;
    }

    /*
     * escape가 조기 종료된 경우,
     * 실제 escape 수행 비율만큼 realign 시간도 줄인다.
     *
     * 예:
     * plannedEscapeMs  = 1500
     * plannedRealignMs = 1800
     * actualEscapeMs   = 500
     *
     * scaledRealignMs  = 1800 * 500 / 1500 = 600
     */
    scaledRealignMs =
        (plannedRealignMs * actualEscapeMs) / plannedEscapeMs;

    /*
     * 너무 짧은 realign은 조향 정렬이 부족할 수 있으므로
     * 최소 시간을 보장한다.
     */
    if(scaledRealignMs < APP_AUTO_EXIT_AVOID_REALIGN_MIN_MS)
    {
        scaledRealignMs = APP_AUTO_EXIT_AVOID_REALIGN_MIN_MS;
    }

    /*
     * 방어 코드.
     * 비율 계산 결과가 계획 realign보다 커지지 않게 제한한다.
     */
    if(scaledRealignMs > plannedRealignMs)
    {
        scaledRealignMs = plannedRealignMs;
    }

    g_autoExit.avoid.plan.realignMs = scaledRealignMs;
}

/*
 * 회피 escape 종료
 *
 * 실제 escape 수행 시간을 저장하고,
 * 변속/방향 전환 안정화를 위해 잠깐 정지 상태로 들어간다.
 */
static void AppAutoExitService_FinishAvoidEscape(void)
{
    g_autoExit.avoid.escapeElapsedMs =
        AppAutoExitService_GetElapsedMs(g_autoExit.avoid.escapeStartTick);

    /*
     * escape가 조기 종료된 경우,
     * 실제 escape 수행 시간에 맞춰 realign 시간도 줄인다.
     */
    AppAutoExitService_AdjustRealignTimeByEscapeElapsed();

    g_autoExit.state = APP_AUTO_EXIT_STATE_AVOID_STOP_1;
    g_autoExit.stateStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_STOP,
                                  APP_AUTO_EXIT_STEER_CENTER);
}

/*
 * 회피 realign 시작
 *
 * escape 후 다시 원래 출차 방향으로 조향하면서 정렬한다.
 */
static void AppAutoExitService_StartAvoidRealign(void)
{
    g_autoExit.avoid.realignStartTick = xTaskGetTickCount();
    g_autoExit.state = APP_AUTO_EXIT_STATE_AVOID_REALIGN;

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_FORWARD,
                                  g_autoExit.avoid.plan.realignSteerCmd);
}

/*
 * 회피 realign 종료
 *
 * 이후 바로 profile을 시작하지 않고,
 * 잠깐 정지 후 기존 출차 profile을 재개한다.
 */
static void AppAutoExitService_FinishAvoidRealign(void)
{
    g_autoExit.state = APP_AUTO_EXIT_STATE_AVOID_STOP_2;
    g_autoExit.stateStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_STOP,
                                  APP_AUTO_EXIT_STEER_CENTER);
}

/*
 * 자동출차 시작
 *
 * command를 direction으로 변환한 뒤 이 함수가 호출된다.
 *
 * 직진 출차:
 *  - 전략 선택 없이 바로 직진 profile 실행
 *
 * 좌/우 출차:
 *  - 먼저 START_STOP 상태로 들어가 잠깐 정지
 *  - 이후 SELECT_STRATEGY에서 NORMAL / AVOID / BLOCKED 판단
 */
static void AppAutoExitService_StartAutoExit(AppAutoExitDirection direction)
{
    /*
     * 이미 자동출차 중이면 새 시작 명령은 무시
     */
    if(g_autoExit.state != APP_AUTO_EXIT_STATE_IDLE)
    {
        return;
    }

    /*
     * Monitor 시작:
     *  - 상태 IN_PROGRESS
     *  - 시작 yaw 저장
     *  - 목표 yaw 계산
     */
    AppAutoExitMonitor_Start(direction);

    g_autoExit.direction = direction;

    /*
     * 직진 출차는 회피 전략 선택 없이 바로 profile 실행
     */
    if(direction == APP_AUTO_EXIT_DIR_STRAIGHT)
    {
        AppAutoExitService_StartProfileForDirection(APP_AUTO_EXIT_DIR_STRAIGHT,
                                                    0u);
        return;
    }

    /*
     * 좌/우 출차는 시작 전에 잠깐 정지 후 전략 선택으로 넘어간다.
     */
    g_autoExit.active = TRUE;
    g_autoExit.state = APP_AUTO_EXIT_STATE_START_STOP;
    g_autoExit.stateStartTick = xTaskGetTickCount();

    AppAutoExitService_SetCommand(APP_AUTO_EXIT_DRIVE_STOP,
                                  APP_AUTO_EXIT_STEER_CENTER);
}

/*
 * 회피 동작을 수행한 뒤,
 * 기존 출차 profile의 첫 번째 전진 시간을 얼마나 줄일지 계산
 *
 * 이유:
 *  - AVOID_ESCAPE와 AVOID_REALIGN 중에도 차량이 어느 정도 전진함
 *  - 그 상태에서 기존 첫 번째 FORWARD step을 그대로 수행하면
 *    너무 많이 앞으로 나갈 수 있음
 *
 * 계산:
 *  - 실제 escape 시간의 일부 비율
 *  - realign 시간의 일부 비율
 * 을 더해서 첫 번째 전진 step 감소량으로 사용한다.
 */
static uint32 AppAutoExitService_CalcFirstStepReductionMs(uint32 escapeMs,
                                                          uint32 realignMs)
{
    uint32 reductionMs;

    reductionMs =
        ((escapeMs * APP_AUTO_EXIT_AVOID_ESCAPE_FORWARD_RATIO_PERCENT) / 100u) +
        ((realignMs * APP_AUTO_EXIT_AVOID_REALIGN_FORWARD_RATIO_PERCENT) / 100u);

    /*
     * 감소량이 첫 번째 전진 시간보다 크거나 같으면,
     * 첫 번째 step이 완전히 사라지지 않도록 최소 시간만 남긴다.
     */
    if(reductionMs >= APP_AUTO_EXIT_FORWARD_1_MS)
    {
        return APP_AUTO_EXIT_FORWARD_1_MS - APP_AUTO_EXIT_MIN_STEP_MS;
    }

    return reductionMs;
}

/*
 * 회피 후 기존 출차 profile 재개
 *
 * 회피 과정에서 이미 어느 정도 앞으로 움직였기 때문에,
 * 첫 번째 전진 step 시간을 일부 줄여서 profile을 시작한다.
 */
static void AppAutoExitService_StartResumeExitProfile(void)
{
    uint32 firstStepReductionMs;

    firstStepReductionMs =
        AppAutoExitService_CalcFirstStepReductionMs(g_autoExit.avoid.escapeElapsedMs, g_autoExit.avoid.plan.realignMs);

    AppAutoExitService_StartProfileForDirection(g_autoExit.direction, firstStepReductionMs);
}

/*
 * 자동출차 상태머신 실행
 *
 * AppAutoExitService_Task()에서 주기적으로 호출된다.
 */
static void AppAutoExitService_ServiceState(void)
{
    AppAutoExitStrategy strategy;
    AppAutoExitAvoidPlan avoidPlan;

    switch(g_autoExit.state)
    {
        case APP_AUTO_EXIT_STATE_IDLE:
            /*
             * 대기 상태에서는 아무 것도 하지 않음
             */
            break;

        case APP_AUTO_EXIT_STATE_START_STOP:
            /*
             * 자동출차 시작 직후 잠깐 정지
             * 차량이 이전 명령의 관성으로 움직이는 것을 줄이기 위한 상태
             */
            if(AppAutoExitService_HasElapsed(g_autoExit.stateStartTick,
                                             APP_AUTO_EXIT_START_STOP_MS) == TRUE)
            {
                g_autoExit.state = APP_AUTO_EXIT_STATE_SELECT_STRATEGY;
            }
            break;

        case APP_AUTO_EXIT_STATE_SELECT_STRATEGY:
            /*
             * Planner에게 현재 주변 상태를 보고
             * NORMAL / AVOID_AND_RESUME / BLOCKED 중 하나를 선택하게 한다.
             */
            strategy = AppAutoExitPlanner_SelectStrategy(g_autoExit.direction,
                                                         &avoidPlan);
            g_autoExit.avoid.plan = avoidPlan;

            if(strategy == APP_AUTO_EXIT_STRATEGY_NORMAL)
            {
                AppAutoExitService_StartProfileForDirection(g_autoExit.direction, 0u);
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
        {
            AppAutoExitAvoidObstacleState obstacleState;

            obstacleState =
                AppAutoExitPlanner_GetAvoidObstacleState(
                    g_autoExit.direction,
                    APP_AUTO_EXIT_AVOID_PHASE_ESCAPE);

            /*
             * ESCAPE 중 DANGER:
             * - 최소 escape 시간을 기다리지 않는다.
             * - 더 밀면 위험하므로 즉시 escape를 끝내고 realign으로 넘어간다.
             */
            if(obstacleState == APP_AUTO_EXIT_AVOID_OBSTACLE_DANGER)
            {
                AppAutoExitService_FinishAvoidEscape();
            }
            /*
             * ESCAPE 중 NEAR:
             * - DANGER 전 단계이므로 조기 종료 대상
             * - 단, escape 시작 직후 바로 끝나는 것을 막기 위해 최소 시간은 보장한다.
             */
            else if((AppAutoExitService_HasElapsed(g_autoExit.avoid.escapeStartTick,
                                                   APP_AUTO_EXIT_AVOID_ESCAPE_MIN_MS) == TRUE) &&
                    (obstacleState == APP_AUTO_EXIT_AVOID_OBSTACLE_NEAR))
            {
                AppAutoExitService_FinishAvoidEscape();
            }
            /*
             * 장애물이 가까워지지 않아도 계획된 escape 시간이 끝나면 realign으로 간다.
             */
            else if(AppAutoExitService_HasElapsed(g_autoExit.avoid.escapeStartTick,
                                                  g_autoExit.avoid.plan.escapeMs) == TRUE)
            {
                AppAutoExitService_FinishAvoidEscape();
            }
            else
            {
                /* Keep escaping */
            }
            break;
        }

        case APP_AUTO_EXIT_STATE_AVOID_STOP_1:
            /*
             * escape 후 잠깐 정지
             * 조향/구동 방향 전환 전 안정화 시간
             */
            if(AppAutoExitService_HasElapsed(g_autoExit.stateStartTick,
                                             APP_AUTO_EXIT_SHIFT_STOP_MS) == TRUE)
            {
                AppAutoExitService_StartAvoidRealign();
            }
            break;

        case APP_AUTO_EXIT_STATE_AVOID_REALIGN:
        {
            AppAutoExitAvoidObstacleState obstacleState;

            obstacleState = AppAutoExitPlanner_GetAvoidObstacleState(g_autoExit.direction, APP_AUTO_EXIT_AVOID_PHASE_REALIGN);

            /*
             * REALIGN 중에는 NEAR를 조기 종료 조건으로 쓰지 않는다.
             * DANGER일 때만 BLOCKED로 간다.
             */
            if(obstacleState == APP_AUTO_EXIT_AVOID_OBSTACLE_DANGER)
            {
                AppAutoExitService_EnterBlocked();
            }
            else if(AppAutoExitService_HasElapsed(g_autoExit.avoid.realignStartTick,
                                                  g_autoExit.avoid.plan.realignMs) == TRUE)
            {
                AppAutoExitService_FinishAvoidRealign();
            }
            else
            {
                /* Keep realigning */
            }
            break;
        }

        case APP_AUTO_EXIT_STATE_AVOID_STOP_2:
            /*
             * realign 후 잠깐 정지한 뒤,
             * 기존 출차 profile을 재개한다.
             */
            if(AppAutoExitService_HasElapsed(g_autoExit.stateStartTick,
                                             APP_AUTO_EXIT_SHIFT_STOP_MS) == TRUE)
            {
                AppAutoExitService_StartResumeExitProfile();
            }
            break;

        case APP_AUTO_EXIT_STATE_RUN_PROFILE:
            /*
             * motion profile step 진행
             */
            AppAutoExitService_ServiceProfile();
            break;

        case APP_AUTO_EXIT_STATE_BLOCKED:
        case APP_AUTO_EXIT_STATE_STOPPED:
            /*
             * BLOCKED 또는 STOPPED 상태에서는
             * 일정 시간 정지 명령을 유지한 뒤 IDLE로 복귀한다.
             */
            if(AppAutoExitService_HasElapsed(g_autoExit.stateStartTick,
                                             APP_AUTO_EXIT_FINAL_STOP_MS) == TRUE)
            {
                AppAutoExitService_EnterIdle();
            }
            break;

        default:
            /*
             * 정의되지 않은 상태가 들어오면 안전하게 BLOCKED 처리
             */
            AppAutoExitService_EnterBlocked();
            break;
    }
}

/*
 * 외부에서 들어온 자동출차 명령 처리
 *
 * 0x300 AutoParking cmd를 받아
 * 실제 자동출차 시작/정지 동작으로 변환한다.
 */
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
            /*
             * 동작 중 STOP이 들어오면 STOPPED 상태로 전환
             * 이미 IDLE이면 Monitor만 IDLE로 정리
             */
            if(g_autoExit.state != APP_AUTO_EXIT_STATE_IDLE)
            {
                AppAutoExitService_EnterStopped();
            }
            else
            {
                AppAutoExitMonitor_SetIdle();
            }
            break;

        case APP_AUTO_EXIT_CMD_NORMAL:
            /*
             * NORMAL은 자동출차 시작 명령이 아니다.
             * IDLE 상태에서 들어오면 Monitor도 IDLE로 맞춘다.
             */
            if(g_autoExit.state == APP_AUTO_EXIT_STATE_IDLE)
            {
                AppAutoExitMonitor_SetIdle();
            }
            break;

        default:
            /*
             * 알 수 없는 명령은 무시
             */
            break;
    }
}

/*
 * 자동출차 서비스 초기화
 */
void AppAutoExitService_Init(void)
{
    AppAutoExitMonitor_Init();

    AppAutoExitService_EnterIdle();

    /*
     * 초기 제어 명령은 정지/중앙 조향
     */
    g_autoExit.cmd.driveCmd = APP_AUTO_EXIT_DRIVE_STOP;
    g_autoExit.cmd.steeringCmd = APP_AUTO_EXIT_STEER_CENTER;
    g_autoExit.lastCommand = APP_AUTO_EXIT_CMD_NORMAL;
}

/*
 * 현재 자동출차 제어가 활성화되어 있는지 반환
 *
 * DriveService나 PDWService에서
 * 자동출차 중인지 판단할 때 사용할 수 있다.
 */
boolean AppAutoExitService_IsActive(void)
{
    return g_autoExit.active;
}

/*
 * 현재 자동출차 제어 명령을 외부로 제공
 *
 * DriveService는 이 함수를 호출해서
 * 자동출차 명령이 있으면 수동 입력 대신 이 명령을 0x100으로 송신한다.
 */
BaseType_t AppAutoExitService_GetControlCommand(VehicleControlCmd_t *cmd)
{
    if(cmd == NULL)
    {
        return pdFAIL;
    }

    /*
     * 자동출차가 비활성 상태이거나
     * 아직 유효한 명령이 없으면 실패 반환
     */
    if((g_autoExit.active == FALSE) || (g_autoExit.cmdValid == FALSE))
    {
        return pdFAIL;
    }

    *cmd = g_autoExit.cmd;
    return pdPASS;
}

/*
 * 자동출차 서비스 Task
 *
 * 주기적으로:
 *  1. 0x300 AutoParking 명령 확인
 *  2. 명령이 바뀌었으면 HandleCommand 수행
 *  3. 상태머신 진행
 *  4. Monitor 상태 송신/관리
 */
void AppAutoExitService_Task(void *arg)
{
    TickType_t lastWakeTime;
    AppAutoParkingState autoParking;

    (void)arg;

    lastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        /*
         * 최신 AutoParking 명령 읽기
         *
         * 같은 명령이 주기적으로 계속 들어오는 경우,
         * lastCommand와 비교해서 명령이 바뀐 순간에만 처리한다.
         */
        if(AppRxService_GetAutoParkingState(&autoParking) == pdPASS)
        {
            if(autoParking.cmd != g_autoExit.lastCommand)
            {
                AppAutoExitService_HandleCommand(autoParking.cmd);
                g_autoExit.lastCommand = autoParking.cmd;
            }
        }

        /*
         * 자동출차 상태머신 실행
         */
        AppAutoExitService_ServiceState();

        /*
         * 0x401 상태 송신 및 yaw 검증 상태 갱신
         */
        AppAutoExitMonitor_Service();

        vTaskDelayUntil(&lastWakeTime,
                        pdMS_TO_TICKS(APP_AUTO_EXIT_SERVICE_PERIOD_MS));
    }
}
