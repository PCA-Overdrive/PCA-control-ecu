#include "App_AutoExitService.h"

#include "App_RxService.h"
#include "task.h"

#define APP_AUTO_EXIT_SERVICE_PERIOD_MS 10u

static boolean g_autoExitActive = FALSE;
static boolean g_autoExitCmdValid = FALSE;
static VehicleControlCmd_t g_autoExitCmd;

void AppAutoExitService_Init(void)
{
    g_autoExitActive = FALSE;
    g_autoExitCmdValid = FALSE;

    g_autoExitCmd.driveCmd = 127u;
    g_autoExitCmd.steeringCmd = 127u;
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
                switch(autoParking.cmd)
                {
                    case APP_AUTO_EXIT_CMD_START_STRAIGHT:
                        /*
                         * 아직 진짜 출차 로직 넣기 전.
                         * 1차 테스트: 직진 출차 명령을 받으면 AutoExit active만 켜고 정지 명령 유지.
                         */
                        g_autoExitActive = TRUE;
                        g_autoExitCmdValid = TRUE;
                        g_autoExitCmd.driveCmd = 127u;
                        g_autoExitCmd.steeringCmd = 127u;
                        break;

                    case APP_AUTO_EXIT_CMD_START_LEFT:
                        /*
                         * 아직 진짜 출차 로직 넣기 전.
                         * 1차 테스트: 왼쪽 출차 명령을 받으면 AutoExit active만 켜고 정지 명령 유지.
                         */
                        g_autoExitActive = TRUE;
                        g_autoExitCmdValid = TRUE;
                        g_autoExitCmd.driveCmd = 127u;
                        g_autoExitCmd.steeringCmd = 127u;
                        break;

                    case APP_AUTO_EXIT_CMD_START_RIGHT:
                        /*
                         * 아직 진짜 출차 로직 넣기 전.
                         * 1차 테스트: 오른쪽 출차 명령을 받으면 AutoExit active만 켜고 정지 명령 유지.
                         */
                        g_autoExitActive = TRUE;
                        g_autoExitCmdValid = TRUE;
                        g_autoExitCmd.driveCmd = 127u;
                        g_autoExitCmd.steeringCmd = 127u;
                        break;

                    case APP_AUTO_EXIT_CMD_STOP:
                        /*
                         * 자동출차 중단 명령.
                         * 현재는 active 해제하고 정지 명령으로 초기화.
                         */
                        g_autoExitActive = FALSE;
                        g_autoExitCmdValid = FALSE;
                        g_autoExitCmd.driveCmd = 127u;
                        g_autoExitCmd.steeringCmd = 127u;
                        break;

                    case APP_AUTO_EXIT_CMD_NORMAL:
                    default:
                        /*
                         * Normal은 별도 동작 없음.
                         * 기존 수동/RPi 입력은 DriveService가 처리.
                         */
                        break;
                }

                prevCmd = autoParking.cmd;
            }
        }

        vTaskDelayUntil(&lastWakeTime,
                        pdMS_TO_TICKS(APP_AUTO_EXIT_SERVICE_PERIOD_MS));
    }
}
