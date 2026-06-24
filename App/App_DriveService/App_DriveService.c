#include "App_DriveService.h"
#include "App_AutoExitService/App_AutoExitService.h"

#include "App_Can.h"
#include "App_PdwService.h"
#include "App_RxService.h"

#include "task.h"

#define APP_DRIVE_SERVICE_PERIOD_MS    (12u)

void AppDriveService_Task(void *arg)
{
    AppPdwState pdw;
    AppRpiInputState rpiInput;
    VehicleControlCmd_t tx;
    VehicleControlCmd_t autoExitCmd;

    (void)arg;

    for(;;)
    {
        if((AppPdwService_GetState(&pdw) == pdPASS) &&
           (AppRxService_GetRpiInput(&rpiInput) == pdPASS))
        {
            if(AppAutoExitService_GetControlCommand(&autoExitCmd) == pdPASS)
            {
                tx = autoExitCmd;
            }
            else
            {
                tx.driveCmd = rpiInput.driveCmd;
                tx.steeringCmd = rpiInput.steeringCmd;
            }

            if((pdw.enabled == TRUE) && (pdw.dangerDetected == TRUE))
            {
                tx.driveCmd = 127u;
            }

            (void)AppCan_SendVehicleControl(&tx);
        }

        vTaskDelay(pdMS_TO_TICKS(APP_DRIVE_SERVICE_PERIOD_MS));
    }
}
