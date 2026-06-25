#include "App_AutoExitService_Internal.h"

#include "App_PdwService.h"

/*
 * 출차 방향 쪽의 측면 거리 특징값을 정리한 구조체
 *
 * 예를 들어 우측 출차라면:
 *  - frontMm : 오른쪽 앞 측면 초음파 거리
 *  - rearMm  : 오른쪽 뒤 측면 초음파 거리
 *
 * diffMm:
 *  - frontMm - rearMm
 *  - diffMm < 0이면 앞쪽이 더 가까운 상태
 *  - diffMm > 0이면 뒤쪽이 더 가까운 상태
 *
 * frontCornerPredMm:
 *  - 측면 앞 센서보다 더 앞에 있는 실제 앞코너 쪽 거리를
 *    앞/뒤 거리 기울기로 단순 선형 예측한 값
 *
 * isFarSafe:
 *  - 앞쪽, 뒤쪽, 예측 앞코너가 모두 충분히 멀어
 *    기울어져 있어도 NORMAL로 볼 수 있는 상태
 *
 * isFrontCloser / isRearCloser:
 *  - 앞/뒤 거리 차이가 TILT_DIFF 이상일 때만 TRUE
 *  - 애매한 거리 구간에서 기울기 방향을 판단하는 데 사용
 */
typedef struct
{
    uint16 frontMm;
    uint16 rearMm;
    uint16 minMm;

    sint32 diffMm;
    sint32 frontCornerPredMm;

    boolean isFarSafe;
    boolean isFrontCloser;
    boolean isRearCloser;
} AppAutoExitSideInfo;

/*
 * 출차 판단에 필요한 양쪽 측면 정보를 묶은 구조체
 *
 * exitSide:
 *  - 실제로 나가려는 방향의 측면 정보
 *  - NORMAL / AVOID / BLOCKED 판단의 기준
 *
 * oppositeSide:
 *  - AVOID 시 차량이 먼저 피하게 되는 반대 방향의 측면 정보
 *  - 이쪽이 충분히 안전해야 AVOID_AND_RESUME 가능
 */
typedef struct
{
    AppAutoExitSideInfo exitSide;
    AppAutoExitSideInfo oppositeSide;
} AppAutoExitSideContext;

/*
 * 여러 방향 중 하나라도 targetLevel 이상인지 확인
 *
 * 예:
 *  - 전진 중 FRONT / FRONT_LEFT / FRONT_RIGHT 중 하나라도 DANGER인지 확인
 *  - 후진 중 BEHIND / BEHIND_LEFT / BEHIND_RIGHT 중 하나라도 DANGER인지 확인
 *
 * AppPdwLevel enum 값이
 * NO_OBSTACLE < SAFE < CAUTION < NEAR < DANGER
 * 순서라고 가정하고 >= 비교를 사용한다.
 */
static boolean AppAutoExitPlanner_IsAnyLevelAtLeast(
    const AppPdwState *pdw,
    const AppPdwDirection *directions,
    uint32 directionCount,
    AppPdwLevel targetLevel)
{
    uint32 index;

    if(pdw == 0)
    {
        return FALSE;
    }

    for(index = 0u; index < directionCount; index++)
    {
        if(pdw->level[directions[index]] >= targetLevel)
        {
            return TRUE;
        }
    }

    return FALSE;
}

/*
 * 측면 앞/뒤 거리값을 이용해 AppAutoExitSideInfo 생성
 *
 * 이 함수는 단순히 앞/뒤 거리를 저장하는 것이 아니라,
 * 옆 차량 또는 장애물이 내 차와 평행한지, 앞쪽으로 가까워지는지,
 * 뒤쪽으로 가까워지는지를 판단하기 위한 특징값을 만든다.
 *
 * 핵심 계산:
 *  1. minMm
 *     - 앞/뒤 중 더 가까운 거리
 *
 *  2. diffMm = frontMm - rearMm
 *     - diffMm < 0: 앞쪽이 더 가까움
 *     - diffMm > 0: 뒤쪽이 더 가까움
 *
 *  3. frontCornerPredMm
 *     - 측면 앞 센서보다 더 앞에 있는 실제 앞코너 예상 거리
 *     - 옆차가 앞쪽으로 기울어져 있으면 front 센서값보다
 *       실제 앞코너 쪽이 더 가까울 수 있으므로 이를 보정
 *
 *  4. isFarSafe
 *     - 앞/뒤 센서와 예측 앞코너가 모두 FAR_SAFE 이상이면 TRUE
 *
 *  5. isFrontCloser / isRearCloser
 *     - 앞/뒤 거리 차이가 TILT_DIFF 이상일 때만 기울어진 상태로 판단
 */
static AppAutoExitSideInfo AppAutoExitPlanner_MakeSideInfo(uint16 frontMm, uint16 rearMm)
{
    AppAutoExitSideInfo info;

    info.frontMm = frontMm;
    info.rearMm = rearMm;
    info.minMm = (frontMm < rearMm) ? frontMm : rearMm;

    /*
     * diffMm < 0:
     * - frontMm이 rearMm보다 작음
     * - 앞쪽으로 갈수록 더 좁아지는 형태
     *
     * diffMm > 0:
     * - rearMm이 frontMm보다 작음
     * - 앞쪽으로 갈수록 공간이 열리는 형태
     */
    info.diffMm = (sint32)frontMm - (sint32)rearMm;

    /*
     * 측면 앞/뒤 초음파 거리 차이를 이용해
     * front sensor보다 더 앞에 있는 실제 앞코너 거리값을 단순 선형 외삽한다.
     *
     * 가정:
     *  - 옆 차량 또는 장애물의 측면이 대략 직선 형태라고 본다.
     *  - front/rear 초음파는 같은 측면을 바라보고 있다고 본다.
     *
     * diffMm < 0:
     *  - 앞쪽 센서가 뒤쪽 센서보다 더 가까움
     *  - 앞코너로 갈수록 거리가 더 작아질 수 있음
     *
     * diffMm > 0:
     *  - 뒤쪽 센서가 앞쪽 센서보다 더 가까움
     *  - 앞코너 방향으로는 공간이 열리는 형태
     */
    info.frontCornerPredMm =
        (sint32)frontMm +
        ((info.diffMm * (sint32)APP_AUTO_EXIT_SIDE_FRONT_OVERHANG_MM) /
         (sint32)APP_AUTO_EXIT_SIDE_SENSOR_SPACING_MM);

    if(info.frontCornerPredMm < 0)
    {
        info.frontCornerPredMm = 0;
    }

    info.isFarSafe =
        ((frontMm >= APP_AUTO_EXIT_SIDE_FAR_SAFE_MM) &&
         (rearMm >= APP_AUTO_EXIT_SIDE_FAR_SAFE_MM) &&
         (info.frontCornerPredMm >= (sint32)APP_AUTO_EXIT_SIDE_FAR_SAFE_MM)) ? TRUE : FALSE;

    if(info.diffMm < -(sint32)APP_AUTO_EXIT_SIDE_TILT_DIFF_MM)
    {
        info.isFrontCloser = TRUE;
        info.isRearCloser = FALSE;
    }
    else if(info.diffMm > (sint32)APP_AUTO_EXIT_SIDE_TILT_DIFF_MM)
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

/*
 * 출차 방향 측면의 위험 형태를 분류한다.
 *
 * 이 함수의 목적은 NORMAL / AVOID / BLOCKED를 직접 결정하는 것이 아니라,
 * 출차 방향 측면이 왜 위험한지를 분류하는 것이다.
 *
 * 판단 개념:
 *  1. 너무 가까우면 CRITICAL
 *  2. 앞/뒤/예측 앞코너가 모두 충분히 멀면 SAFE
 *  3. 앞/뒤가 모두 가까우면 NARROW_BOTH
 *  4. 앞쪽 자체가 가까우면 NEAR_FRONT
 *  5. 뒤쪽 자체가 가까우면 NEAR_REAR
 *  6. 센서값 자체는 괜찮아 보여도 예측 앞코너가 가까우면 TILTED_FRONT
 *  7. 애매한 거리 구간에서 앞쪽으로 좁아지는 기울기면 TILTED_FRONT
 *  8. 애매한 거리 구간에서 뒤쪽이 더 가까워 앞쪽이 열리는 형태면 TILTED_REAR
 *
 * SIDE_RISK_SAFE:
 *  - 기울기가 있어도 충분히 멀어 NORMAL 가능
 *
 * SIDE_RISK_TILTED_FRONT:
 *  - 앞쪽으로 갈수록 공간이 좁아지는 형태
 *  - 앞으로 조향해 나갈 때 앞코너 간섭 가능성이 커서 LONG 회피 대상
 *
 * SIDE_RISK_TILTED_REAR:
 *  - 뒤쪽이 더 가까우나 앞쪽 방향은 열려 있는 형태
 *  - 짧은 회피로 뒤쪽 간섭만 줄이면 되는 SHORT 회피 대상
 */
static AppAutoExitSideRisk AppAutoExitPlanner_GetSideRisk(
    const AppAutoExitSideInfo *side)
{
    if(side == 0)
    {
        /*
         * 내부 static 함수라 NULL이 들어올 가능성은 낮지만,
         * 안전 쪽으로 보수적으로 판단한다.
         */
        return APP_AUTO_EXIT_SIDE_RISK_CRITICAL;
    }

    /*
     * 1. 너무 가까움
     */
    if(side->minMm < APP_AUTO_EXIT_SIDE_CRITICAL_MM)
    {
        return APP_AUTO_EXIT_SIDE_RISK_CRITICAL;
    }

    /*
     * 2. 둘 다 충분히 멀면 기울기 있어도 NORMAL
     */
    if(side->isFarSafe == TRUE)
    {
        return APP_AUTO_EXIT_SIDE_RISK_SAFE;
    }

    /*
     * 3. 앞/뒤 둘 다 가까운 좁은 공간
     */
    if((side->frontMm < APP_AUTO_EXIT_SIDE_NEAR_MM) &&
       (side->rearMm < APP_AUTO_EXIT_SIDE_NEAR_MM))
    {
        return APP_AUTO_EXIT_SIDE_RISK_NARROW_BOTH;
    }

    /*
     * 4. 앞쪽 자체가 가까움
     */
    if(side->frontMm < APP_AUTO_EXIT_SIDE_NEAR_MM)
    {
        return APP_AUTO_EXIT_SIDE_RISK_NEAR_FRONT;
    }

    /*
     * 5. 뒤쪽 자체가 가까움
     */
    if(side->rearMm < APP_AUTO_EXIT_SIDE_NEAR_MM)
    {
        return APP_AUTO_EXIT_SIDE_RISK_NEAR_REAR;
    }

    /*
     * 6. 앞코너 예측 위험
     *
     * front sensor 값은 NEAR 이상이라 괜찮아 보여도,
     * 옆차가 앞쪽으로 기울어진 경우 실제 앞코너 쪽 거리는
     * 더 가까워질 수 있다.
     */
    if(side->frontCornerPredMm < (sint32)APP_AUTO_EXIT_SIDE_NEAR_MM)
    {
        return APP_AUTO_EXIT_SIDE_RISK_TILTED_FRONT;
    }

    /*
     * 7. 애매한 거리 구간에서 기울기 판단
     */
    if(side->isFrontCloser == TRUE)
    {
        return APP_AUTO_EXIT_SIDE_RISK_TILTED_FRONT;
    }

    if(side->isRearCloser == TRUE)
    {
        return APP_AUTO_EXIT_SIDE_RISK_TILTED_REAR;
    }

    return APP_AUTO_EXIT_SIDE_RISK_SAFE;
}

/*
 * SideRisk 결과를 실제 회피량 SHORT / LONG으로 변환한다.
 *
 * LONG:
 *  - 앞쪽이 가깝거나
 *  - 앞쪽으로 갈수록 공간이 좁아지는 기울어진 경우
 *  - 앞/뒤가 모두 좁은 경우
 *  - 너무 가까워 짧은 회피로 부족한 경우
 *
 * SHORT:
 *  - 뒤쪽이 가깝거나
 *  - 애매한 거리 구간에서 뒤쪽이 더 가까워
 *    앞쪽 진행 방향은 열려 있는 경우
 *
 * NONE:
 *  - 기울기가 있어도 충분히 멀거나,
 *    거리 차이가 작아 회피가 필요 없는 경우
 */
static AppAutoExitAvoidLevel AppAutoExitPlanner_GetAvoidLevel(
    const AppAutoExitSideInfo *exitSide)
{
    AppAutoExitSideRisk risk;

    risk = AppAutoExitPlanner_GetSideRisk(exitSide);

    switch(risk)
    {
        case APP_AUTO_EXIT_SIDE_RISK_CRITICAL:
        case APP_AUTO_EXIT_SIDE_RISK_NARROW_BOTH:
        case APP_AUTO_EXIT_SIDE_RISK_NEAR_FRONT:
        case APP_AUTO_EXIT_SIDE_RISK_TILTED_FRONT:
            return APP_AUTO_EXIT_AVOID_LONG;

        case APP_AUTO_EXIT_SIDE_RISK_NEAR_REAR:
        case APP_AUTO_EXIT_SIDE_RISK_TILTED_REAR:
            return APP_AUTO_EXIT_AVOID_SHORT;

        case APP_AUTO_EXIT_SIDE_RISK_SAFE:
        default:
            return APP_AUTO_EXIT_AVOID_NONE;
    }
}

/*
 * 출차 방향 기준으로 측면 context 생성
 *
 * 좌측 출차:
 *  - exitSide     = LEFT_FRONT / LEFT_BEHIND
 *  - oppositeSide = RIGHT_FRONT / RIGHT_BEHIND
 *
 * 우측 출차:
 *  - exitSide     = RIGHT_FRONT / RIGHT_BEHIND
 *  - oppositeSide = LEFT_FRONT / LEFT_BEHIND
 *
 * 여기서는 PDW level이 아니라 distanceMm을 사용한다.
 * 이유:
 *  - 회피 판단은 단순 위험 레벨보다 실제 거리 차이가 중요함
 *  - 옆차 기울기 판단도 앞/뒤 거리 차이로 해야 함
 */
static AppAutoExitSideContext AppAutoExitPlanner_MakeSideContext(
    const AppPdwState *pdw,
    AppAutoExitDirection exitDirection)
{
    AppAutoExitSideContext context;

    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        context.exitSide = AppAutoExitPlanner_MakeSideInfo(
            pdw->distanceMm[APP_PDW_DIR_LEFT_FRONT],
            pdw->distanceMm[APP_PDW_DIR_LEFT_BEHIND]);

        context.oppositeSide = AppAutoExitPlanner_MakeSideInfo(
            pdw->distanceMm[APP_PDW_DIR_RIGHT_FRONT],
            pdw->distanceMm[APP_PDW_DIR_RIGHT_BEHIND]);
    }
    else
    {
        context.exitSide = AppAutoExitPlanner_MakeSideInfo(
            pdw->distanceMm[APP_PDW_DIR_RIGHT_FRONT],
            pdw->distanceMm[APP_PDW_DIR_RIGHT_BEHIND]);

        context.oppositeSide = AppAutoExitPlanner_MakeSideInfo(
            pdw->distanceMm[APP_PDW_DIR_LEFT_FRONT],
            pdw->distanceMm[APP_PDW_DIR_LEFT_BEHIND]);
    }

    return context;
}

/*
 * 회피 level에 따라 실제 회피 시간 plan 설정
 *
 * avoidPlan에는
 *  - escapeMs  : 반대 방향으로 빠져나가는 시간
 *  - realignMs : 다시 원래 출차 방향으로 정렬하는 시간
 * 이 들어간다.
 */
static void AppAutoExitPlanner_SetAvoidPlan(AppAutoExitAvoidPlan *avoidPlan,
                                            AppAutoExitAvoidLevel level)
{
    if(avoidPlan == 0)
    {
        return;
    }

    if(level == APP_AUTO_EXIT_AVOID_LONG)
    {
        /*
         * 긴 회피:
         *  - 앞쪽이 가깝거나
         *  - 앞쪽으로 갈수록 공간이 좁아지는 기울어진 경우
         *  - 앞/뒤가 모두 좁은 경우
         *  - 너무 가까워 짧은 회피로 부족한 경우
         *
         * 즉, 앞으로 조향해 나갈 때 앞코너 또는 차체 진행 경로가
         * 걸릴 가능성이 큰 경우.
         */
        avoidPlan->escapeMs = APP_AUTO_EXIT_AVOID_ESCAPE_LONG_MS;
        avoidPlan->realignMs = APP_AUTO_EXIT_AVOID_REALIGN_LONG_MS;
    }
    else if(level == APP_AUTO_EXIT_AVOID_SHORT)
    {
        /*
         * 짧은 회피:
         *  - 뒤쪽이 가깝거나
         *  - 애매한 거리 구간에서 뒤쪽이 더 가까워
         *    앞쪽 진행 방향은 열려 있는 경우
         *
         * 즉, 앞코너 위험보다는 초기 측면/뒤쪽 간섭을 줄이기 위한 회피.
         */
        avoidPlan->escapeMs = APP_AUTO_EXIT_AVOID_ESCAPE_SHORT_MS;
        avoidPlan->realignMs = APP_AUTO_EXIT_AVOID_REALIGN_SHORT_MS;
    }
    else
    {
        /*
         * 회피 없음
         */
        avoidPlan->escapeMs = 0u;
        avoidPlan->realignMs = 0u;
    }
}

/*
 * 회피 중 반대편이 위험한지 판단
 *
 * 예:
 *  - 좌측으로 출차하려고 오른쪽으로 회피 중이면,
 *    오른쪽 측면과 오른쪽 전방 코너가 위험한지 확인
 *
 * 여기서는 정지 기준을 APP_PDW_LEVEL_DANGER로 본다.
 */
/*
 * 회피 방향의 지정된 PDW level 이상 여부 확인
 *
 * AVOID_ESCAPE 중에는 DANGER까지 기다리면 늦다.
 * 따라서 caller가 targetLevel을 NEAR로 넘기면,
 * 회피 방향이 DANGER가 되기 전에 escape를 끝내고 realign할 수 있다.
 */
static boolean AppAutoExitPlanner_IsAvoidSideLevelAtLeast(
    const AppPdwState *pdw,
    AppPdwDirection sideFrontDirection,
    AppPdwDirection sideRearDirection,
    AppPdwDirection frontCornerDirection,
    AppPdwLevel targetLevel)
{
    if(pdw == 0)
    {
        return FALSE;
    }

    if(pdw->level[sideFrontDirection] >= targetLevel)
    {
        return TRUE;
    }

    if(pdw->level[sideRearDirection] >= targetLevel)
    {
        return TRUE;
    }

    if(pdw->level[frontCornerDirection] >= targetLevel)
    {
        return TRUE;
    }

    return FALSE;
}

/*
 * 현재 motion step을 수행해도 안전한지 확인
 *
 * 이 함수는 자동출차 주행 중 계속 호출되어,
 * 현재 진행 방향 기준으로 DANGER가 있는지 확인한다.
 *
 * 전진 중:
 *  - FRONT
 *  - FRONT_LEFT
 *  - FRONT_RIGHT
 *
 * 후진 중:
 *  - BEHIND
 *  - BEHIND_LEFT
 *  - BEHIND_RIGHT
 *
 * 하나라도 DANGER이면 TRUE를 반환한다.
 * 즉, TRUE는 "위험하다 / 멈춰야 한다"는 의미다.
 */
boolean AppAutoExitPlanner_IsStepSafetyDanger(const AppAutoExitMotionStep *step)
{
    AppPdwState pdw;

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

    /*
     * PDW Service에서 계산된 최신 level/distance 상태를 가져온다.
     */
    if(AppPdwService_GetState(&pdw) != pdPASS)
    {
        return FALSE;
    }

    /*
     * PDW가 비활성 상태이면 안전 판단을 하지 않는다.
     *
     * 단, 현재 App_PdwService 쪽에서 자동출차 중에는
     * pdw.enabled가 TRUE가 되도록 만들어야 이 함수가 의미 있게 동작한다.
     */
    if(pdw.enabled == FALSE)
    {
        return FALSE;
    }

    /*
     * driveCmd가 STOP 기준값보다 작으면 전진으로 판단
     */
    if(step->driveCmd < APP_AUTO_EXIT_DRIVE_STOP)
    {
        return AppAutoExitPlanner_IsAnyLevelAtLeast(&pdw,
                                                    frontDirections,
                                                    APP_AUTO_EXIT_ARRAY_COUNT(frontDirections),
                                                    APP_PDW_LEVEL_DANGER);
    }

    /*
     * driveCmd가 STOP 기준값보다 크면 후진으로 판단
     */
    if(step->driveCmd > APP_AUTO_EXIT_DRIVE_STOP)
    {
        return AppAutoExitPlanner_IsAnyLevelAtLeast(&pdw,
                                                    rearDirections,
                                                    APP_AUTO_EXIT_ARRAY_COUNT(rearDirections),
                                                    APP_PDW_LEVEL_DANGER);
    }

    /*
     * driveCmd가 STOP이면 이동 중이 아니므로 위험 step으로 보지 않는다.
     */
    return FALSE;
}

/*
 * 자동출차 시작 전에 어떤 전략을 사용할지 선택
 *
 * 반환 전략:
 *  - APP_AUTO_EXIT_STRATEGY_NORMAL
 *      그냥 기본 출차 시퀀스 수행
 *
 *  - APP_AUTO_EXIT_STRATEGY_AVOID_AND_RESUME
 *      먼저 반대 방향으로 살짝 회피한 뒤 기본 출차 시퀀스 수행
 *
 *  - APP_AUTO_EXIT_STRATEGY_BLOCKED
 *      전방 또는 출차 방향이 너무 가까워 출차 불가
 *
 * 판단 흐름:
 *  1. 직진 출차면 회피 없이 NORMAL
 *  2. PDW 상태 읽기 실패하면 NORMAL
 *  3. PDW 비활성이면 NORMAL
 *  4. 전방 또는 출차 방향 전방 코너가 DANGER면 BLOCKED
 *
 *  5. 출차 방향 측면 risk 계산
 *     - SAFE이면 기울기가 있어도 충분히 멀다고 보고 NORMAL
 *     - SAFE가 아니면 NORMAL 출차는 위험하거나 애매하다고 판단
 *
 *  6. 출차 방향이 SAFE가 아닌 경우,
 *     반대편이 충분히 안전하면 AVOID_AND_RESUME
 *     - risk 종류에 따라 SHORT / LONG 회피량 결정
 *
 *  7. 출차 방향도 SAFE가 아니고,
 *     반대편도 충분히 안전하지 않으면 BLOCKED
 */
AppAutoExitStrategy AppAutoExitPlanner_SelectStrategy(AppAutoExitDirection exitDirection,
                                                      AppAutoExitAvoidPlan *avoidPlan)
{
    AppPdwState pdw;
    AppAutoExitSideContext sideContext;
    AppAutoExitSideRisk exitRisk;
    AppAutoExitAvoidLevel avoidLevel;

    /*
     * 기본 avoidPlan은 회피 없음으로 초기화
     */
    AppAutoExitPlanner_SetAvoidPlan(avoidPlan, APP_AUTO_EXIT_AVOID_NONE);

    /*
     * 직진 출차는 좌/우 회피 판단 대상이 아니므로 NORMAL
     */
    if(exitDirection == APP_AUTO_EXIT_DIR_STRAIGHT)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    /*
     * PDW 상태를 읽지 못하면 보수적으로 막는 대신,
     * 현재 로직에서는 NORMAL 출차로 진행한다.
     *
     * 안전 우선으로 바꾸려면 BLOCKED 반환을 고려할 수 있다.
     */
    if(AppPdwService_GetState(&pdw) != pdPASS)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    /*
     * PDW가 꺼져 있으면 출차 회피 판단을 하지 않고 NORMAL 진행
     *
     * 자동출차 중에는 App_PdwService 쪽에서 pdw.enabled가 TRUE가 되도록
     * 보장하는 것이 좋다.
     */
    if(pdw.enabled == FALSE)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    /*
     * 출차 방향 기준으로 exitSide / oppositeSide 구성
     */
    sideContext = AppAutoExitPlanner_MakeSideContext(&pdw, exitDirection);

    /*
     * 전방 중앙이 DANGER면 출차 시작 자체가 위험하므로 BLOCKED
     */
    if(pdw.level[APP_PDW_DIR_FRONT] >= APP_PDW_LEVEL_DANGER)
    {
        return APP_AUTO_EXIT_STRATEGY_BLOCKED;
    }

    /*
     * 좌측 출차라면 FRONT_LEFT가 DANGER인지 확인.
     * 나가려는 쪽 앞 코너가 걸릴 수 있기 때문.
     */
    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        if(pdw.level[APP_PDW_DIR_FRONT_LEFT] >= APP_PDW_LEVEL_DANGER)
        {
            return APP_AUTO_EXIT_STRATEGY_BLOCKED;
        }
    }
    else
    {
        /*
         * 우측 출차라면 FRONT_RIGHT가 DANGER인지 확인.
         */
        if(pdw.level[APP_PDW_DIR_FRONT_RIGHT] >= APP_PDW_LEVEL_DANGER)
        {
            return APP_AUTO_EXIT_STRATEGY_BLOCKED;
        }
    }

    exitRisk = AppAutoExitPlanner_GetSideRisk(&sideContext.exitSide);
    avoidLevel = AppAutoExitPlanner_GetAvoidLevel(&sideContext.exitSide);

    /*
     * 출차 방향이 SAFE이면 NORMAL.
     * 여기서 SAFE는 "충분히 멀어서 기울기 있어도 괜찮음"까지 포함한다.
     */
    if(exitRisk == APP_AUTO_EXIT_SIDE_RISK_SAFE)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    /*
     * 출차 방향이 위험하거나 애매한데,
     * 반대편이 충분히 안전하면 AVOID.
     */
    if(sideContext.oppositeSide.isFarSafe == TRUE)
    {
        if(avoidLevel == APP_AUTO_EXIT_AVOID_NONE)
        {
            /*
             * 이론상 exitRisk가 SAFE가 아닌데 avoidLevel이 NONE이면 애매한 상태다.
             * 이 경우 최소한의 보정으로 SHORT 회피를 준다.
             */
            avoidLevel = APP_AUTO_EXIT_AVOID_SHORT;
        }

        AppAutoExitPlanner_SetAvoidPlan(avoidPlan, avoidLevel);
        return APP_AUTO_EXIT_STRATEGY_AVOID_AND_RESUME;
    }

    /*
     * 출차 방향도 SAFE가 아니고,
     * 반대편도 충분히 안전하지 않으면 BLOCKED.
     */
    return APP_AUTO_EXIT_STRATEGY_BLOCKED;
}

/*
 * 회피할 때 사용할 조향값 반환
 *
 * 좌측 출차:
 *  - 바로 좌측으로 나가려는데 왼쪽이 좁으면
 *    먼저 오른쪽으로 피해야 하므로 STEER_RIGHT
 *
 * 우측 출차:
 *  - 바로 우측으로 나가려는데 오른쪽이 좁으면
 *    먼저 왼쪽으로 피해야 하므로 STEER_LEFT
 */
uint8 AppAutoExitPlanner_GetEscapeSteer(AppAutoExitDirection exitDirection)
{
    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        return APP_AUTO_EXIT_STEER_RIGHT;
    }

    return APP_AUTO_EXIT_STEER_LEFT;
}

/*
 * 회피 후 다시 원래 출차 방향으로 복귀할 때 사용할 조향값 반환
 *
 * 좌측 출차:
 *  - 오른쪽으로 회피한 뒤 다시 왼쪽으로 정렬
 *
 * 우측 출차:
 *  - 왼쪽으로 회피한 뒤 다시 오른쪽으로 정렬
 */
uint8 AppAutoExitPlanner_GetRealignSteer(AppAutoExitDirection exitDirection)
{
    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        return APP_AUTO_EXIT_REALIGN_LEFT_STEER;
    }

    return APP_AUTO_EXIT_REALIGN_RIGHT_STEER;
}

/*
 * AVOID_ESCAPE 중 escape를 끝내고 realign으로 넘어가야 하는지 판단
 *
 * AVOID_ESCAPE는 출차 방향이 좁을 때 반대 방향으로 피하는 단계다.
 * 이때 회피 방향이 DANGER가 될 때까지 계속 밀면 너무 늦다.
 *
 * 따라서 회피 방향 측면 또는 회피 방향 앞코너가
 * DANGER 전 단계인 NEAR 이상이 되면,
 * escape를 조기 종료하고 realign으로 넘어간다.
 *
 * 예:
 *  - 좌측 출차:
 *    왼쪽으로 바로 나가기 어려워 오른쪽으로 escape한다.
 *    이때 RIGHT_FRONT / RIGHT_BEHIND / FRONT_RIGHT가 NEAR 이상이면
 *    더 이상 오른쪽으로 밀지 않고 realign한다.
 *
 *  - 우측 출차:
 *    오른쪽으로 바로 나가기 어려워 왼쪽으로 escape한다.
 *    이때 LEFT_FRONT / LEFT_BEHIND / FRONT_LEFT가 NEAR 이상이면
 *    더 이상 왼쪽으로 밀지 않고 realign한다.
 */
boolean AppAutoExitPlanner_ShouldFinishEscapeDuringAvoid(AppAutoExitDirection exitDirection)
{
    AppPdwState pdw;

    if(AppPdwService_GetState(&pdw) != pdPASS)
    {
        return FALSE;
    }

    if(pdw.enabled == FALSE)
    {
        return FALSE;
    }

    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        /*
         * 좌측 출차의 escape 방향은 오른쪽.
         * 오른쪽이 NEAR 이상이면 DANGER가 되기 전에 realign한다.
         */
        return AppAutoExitPlanner_IsAvoidSideLevelAtLeast(&pdw,
                                                          APP_PDW_DIR_RIGHT_FRONT,
                                                          APP_PDW_DIR_RIGHT_BEHIND,
                                                          APP_PDW_DIR_FRONT_RIGHT,
                                                          APP_PDW_LEVEL_NEAR);
    }

    /*
     * 우측 출차의 escape 방향은 왼쪽.
     * 왼쪽이 NEAR 이상이면 DANGER가 되기 전에 realign한다.
     */
    return AppAutoExitPlanner_IsAvoidSideLevelAtLeast(&pdw,
                                                      APP_PDW_DIR_LEFT_FRONT,
                                                      APP_PDW_DIR_LEFT_BEHIND,
                                                      APP_PDW_DIR_FRONT_LEFT,
                                                      APP_PDW_LEVEL_NEAR);
}

/*
 * 회피 동작을 추가했을 때,
 * 기존 첫 번째 전진 시간을 얼마나 줄일지 계산
 *
 * 이유:
 *  - 회피 escape 동작과 realign 동작에서도 차량이 어느 정도 전진/이동할 수 있음
 *  - 그 상태에서 기존 FORWARD_1 시간을 그대로 쓰면 너무 많이 나갈 수 있음
 *  - 그래서 회피 시간의 일정 비율만큼 첫 전진 시간을 줄인다.
 */
uint32 AppAutoExitPlanner_CalcFirstStepReductionMs(uint32 escapeMs,
                                                   uint32 realignMs)
{
    uint32 reductionMs;

    /*
     * escapeMs와 realignMs 중 일부 비율을 전진 시간 감소량으로 환산
     */
    reductionMs =
        ((escapeMs * APP_AUTO_EXIT_AVOID_ESCAPE_FORWARD_RATIO_PERCENT) / 100u) +
        ((realignMs * APP_AUTO_EXIT_AVOID_REALIGN_FORWARD_RATIO_PERCENT) / 100u);

    /*
     * 감소량이 첫 전진 시간보다 크거나 같으면
     * 최소 step 시간은 남기도록 제한한다.
     */
    if(reductionMs >= APP_AUTO_EXIT_FORWARD_1_MS)
    {
        return APP_AUTO_EXIT_FORWARD_1_MS - APP_AUTO_EXIT_MIN_STEP_MS;
    }

    return reductionMs;
}
