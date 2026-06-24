#include "App.h"
#include "App_Can.h"
#include "App_PdwService.h"
#include "App_StatusTxService.h"
#include "App_DriveService.h"
#include "FreeRTOS.h"
#include "task.h"

#define APP_CAN_RX_STACK_SIZE          (configMINIMAL_STACK_SIZE)
#define APP_PDW_STACK_SIZE             (configMINIMAL_STACK_SIZE)
#define APP_STATUS_TX_STACK_SIZE       (configMINIMAL_STACK_SIZE)
#define APP_DRIVE_STACK_SIZE           (configMINIMAL_STACK_SIZE)

#define APP_CAN_RX_PRIORITY            (tskIDLE_PRIORITY + 4u)
#define APP_PDW_PRIORITY               (tskIDLE_PRIORITY + 3u)
#define APP_DRIVE_PRIORITY             (tskIDLE_PRIORITY + 3u)
#define APP_STATUS_TX_PRIORITY         (tskIDLE_PRIORITY + 2u)

static void app_panic_loop(void)
{
    while(1)
    {
        __nop();
    }
}

static void app_assert_pass(BaseType_t result)
{
    if(result != pdPASS)
    {
        app_panic_loop();
    }
}

void App_Init(void)
{
    AppCan_Init();
    AppPdwService_Init();

    app_assert_pass(xTaskCreate(AppCan_RxTask,
                                "CanRx",
                                APP_CAN_RX_STACK_SIZE,
                                NULL,
                                APP_CAN_RX_PRIORITY,
                                NULL));

    app_assert_pass(xTaskCreate(AppPdwService_Task,
                                "Pdw",
                                APP_PDW_STACK_SIZE,
                                NULL,
                                APP_PDW_PRIORITY,
                                NULL));

    app_assert_pass(xTaskCreate(AppDriveService_Task,
                                "Drive",
                                APP_DRIVE_STACK_SIZE,
                                NULL,
                                APP_DRIVE_PRIORITY,
                                NULL));

    app_assert_pass(xTaskCreate(AppStatusTxService_Task,
                                "StatusTx",
                                APP_STATUS_TX_STACK_SIZE,
                                NULL,
                                APP_STATUS_TX_PRIORITY,
                                NULL));
}
