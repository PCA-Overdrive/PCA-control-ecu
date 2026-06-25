#ifndef CAN_MSG_H
#define CAN_MSG_H

#include "Ifx_Types.h"

/* ================================================================
 * 거리 레벨 enum (0x400 공통)
 * ================================================================ */
typedef enum {
    LEVEL_NO_OBSTACLE = 0x00,
    LEVEL_SAFE        = 0x01,
    LEVEL_CAUTION     = 0x02,
    LEVEL_NEAR        = 0x03,
    LEVEL_DANGER      = 0x04
} DistanceLevel_e;

/* ================================================================
 * 0x200 | UltrasonicDistanceCmd
 * Sensor ECU → 판단 ECU | CAN FD | 23 bytes → DLC 12 (24byte 프레임)
 * 전방 1번, 시계방향 10채널
 * ================================================================ */
typedef struct {
    uint16 frontDist;         /* B0~B1   Front          [mm] */
    uint16 frontRightDist;    /* B2~B3   Front-Right    [mm] */
    uint16 rightFrontDist;    /* B4~B5   Right-Front    [mm] */
    uint16 rightBehindDist;   /* B6~B7   Right-Behind   [mm] */
    uint16 behindRightDist;   /* B8~B9   Behind-Right   [mm] */
    uint16 behindDist;        /* B10~B11 Behind         [mm] */
    uint16 behindLeftDist;    /* B12~B13 Behind-Left    [mm] */
    uint16 leftBehindDist;    /* B14~B15 Left-Behind    [mm] */
    uint16 leftFrontDist;     /* B16~B17 Left-Front     [mm] */
    uint16 frontLeftDist;     /* B18~B19 Front-Left     [mm] */
    sint16 imuYaw;            /* B20~B21 IMU Yaw [-180~180]  */
    uint8  vehicleSpeed;      /* B22     속도 [0.1 km/h]     */
} UltrasonicDistanceCmd_t;    /* 23 bytes → DLC 12 (24byte 프레임) */

/* ================================================================
 * 0x400 | DistanceLevelCmd
 * 판단 ECU → 라즈베리파이 | CAN FD | 14 bytes → DLC 10 (16byte 프레임)
 * ================================================================ */
typedef struct {
    uint8 frontLevel;        /* B0  DistanceLevel_e */
    uint8 frontRightLevel;   /* B1  DistanceLevel_e */
    uint8 rightFrontLevel;   /* B2  DistanceLevel_e */
    uint8 rightBehindLevel;  /* B3  DistanceLevel_e */
    uint8 behindRightLevel;  /* B4  DistanceLevel_e */
    uint8 behindLevel;       /* B5  DistanceLevel_e */
    uint8 behindLeftLevel;   /* B6  DistanceLevel_e */
    uint8 leftBehindLevel;   /* B7  DistanceLevel_e */
    uint8 leftFrontLevel;    /* B8  DistanceLevel_e */
    uint8 frontLeftLevel;    /* B9  DistanceLevel_e */
    uint8 pcaActivated;      /* B10 0x00: Not Activated, 0x01: Activated */
    uint8 vehicleSpeed;      /* B11 속도 [0.1 km/h] */
    uint8 gearStatus;        /* B12 GearStatus_e */
    uint8 emergencyStop;     /* B13 0x00: Off, 0x01: On */
} DistanceLevelCmd_t;        /* 14 bytes → DLC 10 (16byte 프레임) */

/* ================================================================
 * 0x401 | ExitCompleteCmd
 * 판단 ECU → 라즈베리파이 | Classical CAN | 1 byte
 * ================================================================ */
typedef enum {
    AUTOPARKING_STATUS_NORMAL   = 0x00,
    AUTOPARKING_STATUS_RUNNING  = 0x01,
    AUTOPARKING_STATUS_COMPLETE = 0x02,
    AUTOPARKING_STATUS_STOPPED  = 0x03
} AutoparkingStatus_e;

typedef struct {
    uint8 autoparkingStatus; /* B0  AutoparkingStatus_e */
} ExitCompleteCmd_t;         /* 1 byte */

/* ================================================================
 * 0x100 | VehicleControlCmd
 * 판단 ECU → Motor ECU | Classical CAN | 2 bytes
 * 0~126: 전진/좌조향, 127: 중립/중앙, 128~255: 후진/우조향
 * ================================================================ */
typedef struct {
    uint8 driveCmd;          /* B0  0~255 주행 명령 */
    uint8 steeringCmd;       /* B1  0~255 조향 명령 */
} VehicleControlCmd_t;

/* ================================================================
 * 0x201 | VehicleStatusCmd
 * 라즈베리파이 → 판단 ECU | Classical CAN | 4 bytes
 * ================================================================ */
typedef enum {
    GEAR_P = 0x00,
    GEAR_D = 0x01,
    GEAR_R = 0x02
} GearStatus_e;

typedef struct {
    uint8 driveCmd;          /* B0  0~255 주행 명령 */
    uint8 steeringCmd;       /* B1  0~255 조향 명령 */
    uint8 gearStatus;        /* B2  GearStatus_e */
    uint8 pcaActivated;      /* B3  0x00: Not Activated, 0x01: Activated */
    sint16 lineAngle;        /* B4~B5  각도 (-180 ~ 180) */
} VehicleStatusCmd_t;        /* 6 bytes */

/* ================================================================
 * 0x300 | AutoParkingCmd
 * 라즈베리파이 → 판단 ECU | Classical CAN | 1 byte
 * ================================================================ */
typedef enum {
    AUTO_PARKING_NORMAL         = 0x00,
    AUTO_PARKING_START_STRAIGHT = 0x01,
    AUTO_PARKING_START_LEFT     = 0x02,
    AUTO_PARKING_START_RIGHT    = 0x03,
    AUTO_PARKING_STOP           = 0x04
} AutoParkingStart_e;

typedef struct {
    uint8 autoParkingStart;  /* B0  AutoParkingStart_e */
} AutoParkingCmd_t;          /* 1 byte */

#endif /* CAN_MSG_H */
