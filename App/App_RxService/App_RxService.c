#include "App_RxService.h"

#include "App_Can.h"
//0x201, 0x200, 0x300 메시지 각각 구조체에 저장
//0x201 메시지 구조체를 AppRpiInputState 구조체로 변환
BaseType_t AppRxService_GetRpiInput(AppRpiInputState *input)
{
    VehicleStatusCmd_t rx;

    if(input == NULL)
    {
        return pdFAIL;
    }
    //mailbox에서 수신된 최신 0x201 메시지 가져오기
    if(AppCan_GetLatestVehicleStatus(&rx) != pdPASS)
    {
        return pdFAIL;
    }
    //0x201 메시지 구조체를 AppRpiInputState 구조체로 변환
    input->driveCmd = rx.driveCmd;
    input->steeringCmd = rx.steeringCmd;
    input->gear = (AppGearStatus)rx.gearStatus;
    input->pdwSwitchOn = (rx.pcaActivated != 0u) ? TRUE : FALSE;

    return pdPASS;
}
//0x200 메시지 구조체를 AppUltrasonicState 구조체로 변환
BaseType_t AppRxService_GetUltrasonicState(AppUltrasonicState *state)
{
    UltrasonicDistanceCmd_t rx;

    if(state == NULL)
    {
        return pdFAIL;
    }

    if(AppCan_GetLatestUltrasonic(&rx) != pdPASS)
    {
        return pdFAIL;
    }

    state->distanceMm[APP_PDW_DIR_FRONT] = rx.frontDist;
    state->distanceMm[APP_PDW_DIR_FRONT_RIGHT] = rx.frontRightDist;
    state->distanceMm[APP_PDW_DIR_RIGHT_FRONT] = rx.rightFrontDist;
    state->distanceMm[APP_PDW_DIR_RIGHT_BEHIND] = rx.rightBehindDist;
    state->distanceMm[APP_PDW_DIR_BEHIND_RIGHT] = rx.behindRightDist;
    state->distanceMm[APP_PDW_DIR_BEHIND] = rx.behindDist;
    state->distanceMm[APP_PDW_DIR_BEHIND_LEFT] = rx.behindLeftDist;
    state->distanceMm[APP_PDW_DIR_LEFT_BEHIND] = rx.leftBehindDist;
    state->distanceMm[APP_PDW_DIR_LEFT_FRONT] = rx.leftFrontDist;
    state->distanceMm[APP_PDW_DIR_FRONT_LEFT] = rx.frontLeftDist;
    state->imuYaw = rx.imuYaw;
    state->vehicleSpeed = rx.vehicleSpeed;

    return pdPASS;
}

BaseType_t AppRxService_GetAutoParkingState(AppAutoParkingState *state)
{
    AutoParkingCmd_t rx;

    if(state == NULL)
    {
        return pdFAIL;
    }

    if(AppCan_GetLatestAutoParking(&rx) != pdPASS)
    {
        return pdFAIL;
    }

    switch(rx.autoParkingStart)
    {
        case AUTO_PARKING_NORMAL:
            state->cmd = APP_AUTO_EXIT_CMD_NORMAL;
            break;

        case AUTO_PARKING_START_STRAIGHT:
            state->cmd = APP_AUTO_EXIT_CMD_START_STRAIGHT;
            break;

        case AUTO_PARKING_START_LEFT:
            state->cmd = APP_AUTO_EXIT_CMD_START_LEFT;
            break;

        case AUTO_PARKING_START_RIGHT:
            state->cmd = APP_AUTO_EXIT_CMD_START_RIGHT;
            break;

        case AUTO_PARKING_STOP:
            state->cmd = APP_AUTO_EXIT_CMD_STOP;
            break;

        default:
            state->cmd = APP_AUTO_EXIT_CMD_NORMAL;
            break;
    }

    return pdPASS;
}
