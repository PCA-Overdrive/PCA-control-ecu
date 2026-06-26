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

    /*
     * AVOID escape 방향의 전방 코너 거리
     *
     * 좌측 출차:
     *  - 오른쪽으로 escape하므로 FRONT_RIGHT
     *
     * 우측 출차:
     *  - 왼쪽으로 escape하므로 FRONT_LEFT
     *
     * oppositeSide의 front/rear에는 측면 센서만 들어가므로,
     * 전방 코너 raw CRITICAL 판단을 위해 별도로 저장한다.
     */
    uint16 oppositeFrontCornerMm;
} AppAutoExitSideContext;

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

        /*
         * 좌측 출차의 escape 방향은 오른쪽이므로
         * 반대편 앞코너는 FRONT_RIGHT.
         */
        context.oppositeFrontCornerMm =
            pdw->distanceMm[APP_PDW_DIR_FRONT_RIGHT];
    }
    else
    {
        context.exitSide = AppAutoExitPlanner_MakeSideInfo(
            pdw->distanceMm[APP_PDW_DIR_RIGHT_FRONT],
            pdw->distanceMm[APP_PDW_DIR_RIGHT_BEHIND]);

        context.oppositeSide = AppAutoExitPlanner_MakeSideInfo(
            pdw->distanceMm[APP_PDW_DIR_LEFT_FRONT],
            pdw->distanceMm[APP_PDW_DIR_LEFT_BEHIND]);

        /*
         * 우측 출차의 escape 방향은 왼쪽이므로
         * 반대편 앞코너는 FRONT_LEFT.
         */
        context.oppositeFrontCornerMm =
            pdw->distanceMm[APP_PDW_DIR_FRONT_LEFT];
    }

    return context;
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

    if(step == 0)
    {
        return FALSE;
    }

    if(AppPdwService_GetState(&pdw) != pdPASS)
    {
        return FALSE;
    }

    if(pdw.enabled == FALSE)
    {
        return FALSE;
    }

    /*
     * 전진 중 안전 체크
     *
     * 기존:
     * - FRONT
     * - FRONT_LEFT
     * - FRONT_RIGHT
     *
     * 보강:
     * - 좌회전 전진이면 LEFT_FRONT도 확인
     * - 우회전 전진이면 RIGHT_FRONT도 확인
     */
    if(step->driveCmd < APP_AUTO_EXIT_DRIVE_STOP)
    {
        if((pdw.level[APP_PDW_DIR_FRONT] >= APP_PDW_LEVEL_DANGER) ||
           (pdw.level[APP_PDW_DIR_FRONT_LEFT] >= APP_PDW_LEVEL_DANGER) ||
           (pdw.level[APP_PDW_DIR_FRONT_RIGHT] >= APP_PDW_LEVEL_DANGER))
        {
            return TRUE;
        }

        if(step->steeringCmd < APP_AUTO_EXIT_STEER_CENTER)
        {
            if(pdw.level[APP_PDW_DIR_LEFT_FRONT] >= APP_PDW_LEVEL_DANGER)
            {
                return TRUE;
            }
        }
        else if(step->steeringCmd > APP_AUTO_EXIT_STEER_CENTER)
        {
            if(pdw.level[APP_PDW_DIR_RIGHT_FRONT] >= APP_PDW_LEVEL_DANGER)
            {
                return TRUE;
            }
        }

        return FALSE;
    }

    /*
     * 후진 중 안전 체크
     *
     * 기존:
     * - BEHIND
     * - BEHIND_LEFT
     * - BEHIND_RIGHT
     *
     * 보강:
     * - 좌회전 후진이면 LEFT_BEHIND도 확인
     * - 우회전 후진이면 RIGHT_BEHIND도 확인
     */
    if(step->driveCmd > APP_AUTO_EXIT_DRIVE_STOP)
    {
        if((pdw.level[APP_PDW_DIR_BEHIND] >= APP_PDW_LEVEL_DANGER) ||
           (pdw.level[APP_PDW_DIR_BEHIND_LEFT] >= APP_PDW_LEVEL_DANGER) ||
           (pdw.level[APP_PDW_DIR_BEHIND_RIGHT] >= APP_PDW_LEVEL_DANGER))
        {
            return TRUE;
        }

        if(step->steeringCmd < APP_AUTO_EXIT_STEER_CENTER)
        {
            if(pdw.level[APP_PDW_DIR_LEFT_BEHIND] >= APP_PDW_LEVEL_DANGER)
            {
                return TRUE;
            }
        }
        else if(step->steeringCmd > APP_AUTO_EXIT_STEER_CENTER)
        {
            if(pdw.level[APP_PDW_DIR_RIGHT_BEHIND] >= APP_PDW_LEVEL_DANGER)
            {
                return TRUE;
            }
        }

        return FALSE;
    }

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
 *     반대편이 CRITICAL 또는 DANGER가 아니면 AVOID_AND_RESUME
 *     - 반대편이 완전히 넓을 필요는 없음
 *     - risk 종류에 따라 SHORT / LONG 회피량 결정
 *
 *  7. 출차 방향도 SAFE가 아니고,
 *     반대편도 CRITICAL 또는 DANGER이면 BLOCKED
 */
AppAutoExitStrategy AppAutoExitPlanner_SelectStrategy(AppAutoExitDirection exitDirection,
                                                      AppAutoExitAvoidPlan *avoidPlan)
{
    AppPdwState pdw;
    AppAutoExitSideContext sideContext;
    AppAutoExitSideRisk exitRisk;
    AppAutoExitAvoidLevel avoidLevel;
    boolean escapeSideDanger;

    if(avoidPlan != 0)
    {
        avoidPlan->escapeMs = 0u;
        avoidPlan->realignMs = 0u;
        avoidPlan->escapeSteerCmd = APP_AUTO_EXIT_STEER_CENTER;
        avoidPlan->realignSteerCmd = APP_AUTO_EXIT_STEER_CENTER;
    }

    if(exitDirection == APP_AUTO_EXIT_DIR_STRAIGHT)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    if(AppPdwService_GetState(&pdw) != pdPASS)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    if(pdw.enabled == FALSE)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    sideContext = AppAutoExitPlanner_MakeSideContext(&pdw, exitDirection);

    if(pdw.level[APP_PDW_DIR_FRONT] >= APP_PDW_LEVEL_DANGER)
    {
        return APP_AUTO_EXIT_STRATEGY_BLOCKED;
    }

    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        if(pdw.level[APP_PDW_DIR_FRONT_LEFT] >= APP_PDW_LEVEL_DANGER)
        {
            return APP_AUTO_EXIT_STRATEGY_BLOCKED;
        }
    }
    else
    {
        if(pdw.level[APP_PDW_DIR_FRONT_RIGHT] >= APP_PDW_LEVEL_DANGER)
        {
            return APP_AUTO_EXIT_STRATEGY_BLOCKED;
        }
    }

    exitRisk = AppAutoExitPlanner_GetSideRisk(&sideContext.exitSide);

    if(exitRisk == APP_AUTO_EXIT_SIDE_RISK_SAFE)
    {
        return APP_AUTO_EXIT_STRATEGY_NORMAL;
    }

    switch(exitRisk)
    {
        case APP_AUTO_EXIT_SIDE_RISK_CRITICAL:
        case APP_AUTO_EXIT_SIDE_RISK_NARROW_BOTH:
        case APP_AUTO_EXIT_SIDE_RISK_NEAR_FRONT:
        case APP_AUTO_EXIT_SIDE_RISK_TILTED_FRONT:
            avoidLevel = APP_AUTO_EXIT_AVOID_LONG;
            break;

        case APP_AUTO_EXIT_SIDE_RISK_NEAR_REAR:
        case APP_AUTO_EXIT_SIDE_RISK_TILTED_REAR:
            avoidLevel = APP_AUTO_EXIT_AVOID_SHORT;
            break;

        case APP_AUTO_EXIT_SIDE_RISK_SAFE:
        default:
            avoidLevel = APP_AUTO_EXIT_AVOID_SHORT;
            break;
    }

    /*
     * 반대편 측면 또는 반대편 앞코너가 너무 가까우면
     * escape 시작 불가.
     */
    if(sideContext.oppositeSide.minMm < APP_AUTO_EXIT_SIDE_CRITICAL_MM)
    {
        return APP_AUTO_EXIT_STRATEGY_BLOCKED;
    }

    if(sideContext.oppositeFrontCornerMm < APP_AUTO_EXIT_SIDE_CRITICAL_MM)
    {
        return APP_AUTO_EXIT_STRATEGY_BLOCKED;
    }

    /*
     * escape 방향이 이미 DANGER이면 escape 시작 불가.
     */
    if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
    {
        escapeSideDanger =
            ((pdw.level[APP_PDW_DIR_RIGHT_FRONT] >= APP_PDW_LEVEL_DANGER) ||
             (pdw.level[APP_PDW_DIR_RIGHT_BEHIND] >= APP_PDW_LEVEL_DANGER) ||
             (pdw.level[APP_PDW_DIR_FRONT_RIGHT] >= APP_PDW_LEVEL_DANGER)) ? TRUE : FALSE;
    }
    else
    {
        escapeSideDanger =
            ((pdw.level[APP_PDW_DIR_LEFT_FRONT] >= APP_PDW_LEVEL_DANGER) ||
             (pdw.level[APP_PDW_DIR_LEFT_BEHIND] >= APP_PDW_LEVEL_DANGER) ||
             (pdw.level[APP_PDW_DIR_FRONT_LEFT] >= APP_PDW_LEVEL_DANGER)) ? TRUE : FALSE;
    }

    if(escapeSideDanger == TRUE)
    {
        return APP_AUTO_EXIT_STRATEGY_BLOCKED;
    }

    if(avoidPlan != 0)
    {
        if(avoidLevel == APP_AUTO_EXIT_AVOID_LONG)
        {
            avoidPlan->escapeMs = APP_AUTO_EXIT_AVOID_ESCAPE_LONG_MS;
            avoidPlan->realignMs = APP_AUTO_EXIT_AVOID_REALIGN_LONG_MS;
        }
        else
        {
            avoidPlan->escapeMs = APP_AUTO_EXIT_AVOID_ESCAPE_SHORT_MS;
            avoidPlan->realignMs = APP_AUTO_EXIT_AVOID_REALIGN_SHORT_MS;
        }

        if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
        {
            /*
             * 좌측 출차는 먼저 오른쪽으로 escape,
             * 이후 왼쪽으로 realign.
             */
            avoidPlan->escapeSteerCmd = APP_AUTO_EXIT_STEER_RIGHT;
            avoidPlan->realignSteerCmd = APP_AUTO_EXIT_REALIGN_LEFT_STEER;
        }
        else
        {
            /*
             * 우측 출차는 먼저 왼쪽으로 escape,
             * 이후 오른쪽으로 realign.
             */
            avoidPlan->escapeSteerCmd = APP_AUTO_EXIT_STEER_LEFT;
            avoidPlan->realignSteerCmd = APP_AUTO_EXIT_REALIGN_RIGHT_STEER;
        }
    }

    return APP_AUTO_EXIT_STRATEGY_AVOID_AND_RESUME;
}

/*
 * AVOID 동작 중 현재 phase 기준 장애물 상태를 반환한다.
 *
 * 반환값:
 *  - CLEAR  : 해당 방향 센서들이 아직 NEAR/DANGER 아님
 *  - NEAR   : 해당 방향 센서 중 하나라도 NEAR 이상
 *  - DANGER : 해당 방향 센서 중 하나라도 DANGER 이상
 *
 * 사용 정책은 Service에서 phase별로 다르게 적용한다.
 *
 * ESCAPE:
 *  - DANGER이면 최소 시간 기다리지 않고 즉시 realign
 *  - NEAR이면 최소 escape 시간 이후 realign
 *
 * REALIGN:
 *  - DANGER이면 BLOCKED
 *  - NEAR는 무시하고 계속 진행
 */
AppAutoExitAvoidObstacleState AppAutoExitPlanner_GetAvoidObstacleState(
    AppAutoExitDirection exitDirection,
    AppAutoExitAvoidPhase phase)
{
    AppPdwState pdw;
    AppPdwLevel level0;
    AppPdwLevel level1;
    AppPdwLevel level2;

    if(AppPdwService_GetState(&pdw) != pdPASS)
    {
        return APP_AUTO_EXIT_AVOID_OBSTACLE_CLEAR;
    }

    if(pdw.enabled == FALSE)
    {
        return APP_AUTO_EXIT_AVOID_OBSTACLE_CLEAR;
    }

    level0 = APP_PDW_LEVEL_NO_OBSTACLE;
    level1 = APP_PDW_LEVEL_NO_OBSTACLE;
    level2 = APP_PDW_LEVEL_NO_OBSTACLE;

    if(phase == APP_AUTO_EXIT_AVOID_PHASE_ESCAPE)
    {
        if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
        {
            level0 = pdw.level[APP_PDW_DIR_RIGHT_FRONT];
            level1 = pdw.level[APP_PDW_DIR_RIGHT_BEHIND];
            level2 = pdw.level[APP_PDW_DIR_FRONT_RIGHT];
        }
        else if(exitDirection == APP_AUTO_EXIT_DIR_RIGHT)
        {
            level0 = pdw.level[APP_PDW_DIR_LEFT_FRONT];
            level1 = pdw.level[APP_PDW_DIR_LEFT_BEHIND];
            level2 = pdw.level[APP_PDW_DIR_FRONT_LEFT];
        }
        else
        {
            /* Nothing to do */
        }
    }
    else if(phase == APP_AUTO_EXIT_AVOID_PHASE_REALIGN)
    {
        if(exitDirection == APP_AUTO_EXIT_DIR_LEFT)
        {
            level0 = pdw.level[APP_PDW_DIR_LEFT_FRONT];
            level1 = pdw.level[APP_PDW_DIR_LEFT_BEHIND];
            level2 = pdw.level[APP_PDW_DIR_FRONT_LEFT];
        }
        else if(exitDirection == APP_AUTO_EXIT_DIR_RIGHT)
        {
            level0 = pdw.level[APP_PDW_DIR_RIGHT_FRONT];
            level1 = pdw.level[APP_PDW_DIR_RIGHT_BEHIND];
            level2 = pdw.level[APP_PDW_DIR_FRONT_RIGHT];
        }
        else
        {
            /* Nothing to do */
        }
    }
    else
    {
        /* Nothing to do */
    }

    if((level0 >= APP_PDW_LEVEL_DANGER) ||
       (level1 >= APP_PDW_LEVEL_DANGER) ||
       (level2 >= APP_PDW_LEVEL_DANGER))
    {
        return APP_AUTO_EXIT_AVOID_OBSTACLE_DANGER;
    }

    if((level0 >= APP_PDW_LEVEL_NEAR) ||
       (level1 >= APP_PDW_LEVEL_NEAR) ||
       (level2 >= APP_PDW_LEVEL_NEAR))
    {
        return APP_AUTO_EXIT_AVOID_OBSTACLE_NEAR;
    }

    return APP_AUTO_EXIT_AVOID_OBSTACLE_CLEAR;
}
