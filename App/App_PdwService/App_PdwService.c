#include <stddef.h>
#include "App_PdwService.h"
#include "App_RxService.h"
#include "task.h"

#define APP_PDW_SERVICE_PERIOD_MS     (10u)

#define APP_PDW_NO_OBSTACLE_DISTANCE_MM    (0u)
#define APP_PDW_SAFE_THRESHOLD_MM          (1000u)
#define APP_PDW_CAUTION_THRESHOLD_MM       (600u)
#define APP_PDW_NEAR_THRESHOLD_MM          (300u)
#define APP_PDW_DANGER_THRESHOLD_MM        (150u)

static AppPdwState g_pdwState;

static AppPdwLevel AppPdwService_ClassifyDistance(uint16 distanceMm);
static void AppPdwService_ClearState(AppPdwState *state);

void AppPdwService_Init(void)
{
    AppPdwService_ClearState(&g_pdwState);
}

void AppPdwService_Task(void *arg)
{
    AppRpiInputState rpiInput;
    AppUltrasonicState ultrasonic;
    AppAutoParkingState autoParkingState;

    (void)arg;

    for(;;)
    {
        // 수신된 최신 0x201, 0x200 메시지 가져오기
        if((AppRxService_GetRpiInput(&rpiInput) == pdPASS) &&
           (AppRxService_GetUltrasonicState(&ultrasonic) == pdPASS) &&
           (AppRxService_GetAutoParkingState(&autoParkingState) == pdPASS))
        {
            AppPdwService_Process(&rpiInput, &ultrasonic, &autoParkingState);
        }

        vTaskDelay(pdMS_TO_TICKS(APP_PDW_SERVICE_PERIOD_MS));
    }
}

void AppPdwService_Process(const AppRpiInputState *rpiInput,
                           const AppUltrasonicState *ultrasonic,
                            const AppAutoParkingState *autoParkingState)
{
    AppPdwState nextState;
    uint8 i;
    if((rpiInput == NULL) || (ultrasonic == NULL))
    {
        return;
    }
    AppPdwService_ClearState(&nextState);

    // PDW 활성화 여부 결정: RPi에서 PDW 스위치가 켜져 있고, 기어가 P가 아닌 경우 활성화
    nextState.enabled = (
                    ((rpiInput->pdwSwitchOn == TRUE) && (rpiInput->gear != APP_GEAR_P))
                    ||
                    ((autoParkingState != NULL && autoParkingState->cmd != APP_AUTO_EXIT_CMD_STOP && autoParkingState->cmd != APP_AUTO_EXIT_CMD_NORMAL))
                    ) ? TRUE : FALSE;

    for(i = 0u; i < APP_PDW_DIRECTION_COUNT; i++)
    {
        nextState.distanceMm[i] = ultrasonic->distanceMm[i];
        if(nextState.enabled == TRUE)
        {
            nextState.level[i] = AppPdwService_ClassifyDistance(ultrasonic->distanceMm[i]);
            if(nextState.level[i] == APP_PDW_LEVEL_DANGER)
            {
                nextState.dangerDetected = TRUE;
            }
        }
    }

    taskENTER_CRITICAL();
    g_pdwState = nextState;
    taskEXIT_CRITICAL();
}

BaseType_t AppPdwService_GetState(AppPdwState *state)
{
    if(state == NULL)
    {
        return pdFAIL;
    }

    taskENTER_CRITICAL();
    *state = g_pdwState;
    taskEXIT_CRITICAL();

    return pdPASS;
}

static AppPdwLevel AppPdwService_ClassifyDistance(uint16 distanceMm)
{
    if(distanceMm == APP_PDW_NO_OBSTACLE_DISTANCE_MM)
    {
        return APP_PDW_LEVEL_NO_OBSTACLE;
    }

    if(distanceMm <= APP_PDW_DANGER_THRESHOLD_MM)
    {
        return APP_PDW_LEVEL_DANGER;
    }

    if(distanceMm <= APP_PDW_NEAR_THRESHOLD_MM)
    {
        return APP_PDW_LEVEL_NEAR;
    }

    if(distanceMm <= APP_PDW_CAUTION_THRESHOLD_MM)
    {
        return APP_PDW_LEVEL_CAUTION;
    }

    if(distanceMm <= APP_PDW_SAFE_THRESHOLD_MM)
    {
        return APP_PDW_LEVEL_SAFE;
    }

    return APP_PDW_LEVEL_NO_OBSTACLE;
}

static void AppPdwService_ClearState(AppPdwState *state)
{
    uint8 i;

    state->enabled = FALSE;
    state->dangerDetected = FALSE;
    for(i = 0u; i < APP_PDW_DIRECTION_COUNT; i++)
    {
        state->level[i] = APP_PDW_LEVEL_NO_OBSTACLE;
        state->distanceMm[i] = 0u;
    }
}
