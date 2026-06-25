#ifndef APP_TYPES_H
#define APP_TYPES_H

#include "Ifx_Types.h"

#define APP_PDW_DIRECTION_COUNT        (10u)

typedef enum
{
    APP_GEAR_P = 0x00u,
    APP_GEAR_D = 0x01u,
    APP_GEAR_R = 0x02u
} AppGearStatus;

typedef enum
{
    APP_PDW_LEVEL_NO_OBSTACLE = 0x00u,
    APP_PDW_LEVEL_SAFE        = 0x01u,
    APP_PDW_LEVEL_CAUTION     = 0x02u,
    APP_PDW_LEVEL_NEAR        = 0x03u,
    APP_PDW_LEVEL_DANGER      = 0x04u
} AppPdwLevel;

typedef enum
{
    APP_PDW_DIR_FRONT = 0u,
    APP_PDW_DIR_FRONT_RIGHT,
    APP_PDW_DIR_RIGHT_FRONT,
    APP_PDW_DIR_RIGHT_BEHIND,
    APP_PDW_DIR_BEHIND_RIGHT,
    APP_PDW_DIR_BEHIND,
    APP_PDW_DIR_BEHIND_LEFT,
    APP_PDW_DIR_LEFT_BEHIND,
    APP_PDW_DIR_LEFT_FRONT,
    APP_PDW_DIR_FRONT_LEFT
} AppPdwDirection;

typedef enum
{
    APP_AUTO_EXIT_CMD_NORMAL = 0x00u,
    APP_AUTO_EXIT_CMD_START_STRAIGHT = 0x01u,
    APP_AUTO_EXIT_CMD_START_LEFT = 0x02u,
    APP_AUTO_EXIT_CMD_START_RIGHT = 0x03u,
    APP_AUTO_EXIT_CMD_STOP = 0x04u
} AppAutoExitCmd;

//CAN 0x201 message 저장용 구조체
typedef struct
{
    uint8 driveCmd;
    uint8 steeringCmd;
    AppGearStatus gear;
    boolean pdwSwitchOn;
} AppRpiInputState;

//CAN 0x200 message 저장용 구조체
typedef struct
{
    uint16 distanceMm[APP_PDW_DIRECTION_COUNT];
    sint16 imuYaw;
    uint8 vehicleSpeed;
} AppUltrasonicState;

//CAN 0x300 message 저장용 구조체
typedef struct
{
    AppAutoExitCmd cmd;
} AppAutoParkingState;

//PDW 판단 ECU 상태값 -> 나중에 송신할 0x400 메시지 만들 때 StatusTxService에서 사용할 구조체
typedef struct
{
    boolean enabled;
    boolean dangerDetected;
    AppPdwLevel level[APP_PDW_DIRECTION_COUNT];
    uint16 distanceMm[APP_PDW_DIRECTION_COUNT];
} AppPdwState;

#endif /* APP_TYPES_H */
