#ifndef APP_AUTO_EXIT_SERVICE_INTERNAL_H
#define APP_AUTO_EXIT_SERVICE_INTERNAL_H

#include "App_AutoExitConfig.h"
#include "App_AutoExitTypes.h"

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

void AppAutoExitMonitor_Init(void);
void AppAutoExitMonitor_Start(AppAutoExitDirection direction);
void AppAutoExitMonitor_SetIdle(void);
void AppAutoExitMonitor_SetResult(AppAutoExitStatus status);
boolean AppAutoExitMonitor_FinishAndValidate(AppAutoExitDirection direction);
void AppAutoExitMonitor_Service(void);

#endif /* APP_AUTO_EXIT_SERVICE_INTERNAL_H */
