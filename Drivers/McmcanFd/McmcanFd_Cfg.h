#ifndef MCMCANFD_CFG_H
#define MCMCANFD_CFG_H

/* 빌드 시 하나만 활성화 */
// #define NODE_SENSOR_ECU
#define NODE_JUDGMENT_ECU
// #define NODE_MOTOR_ECU

/* 비트레이트 */
#define MCMCAN_ARB_BAUDRATE       500000U
#define MCMCAN_DATA_BAUDRATE     2000000U

#define ISR_PRIORITY_CAN_RX  50U

/* ── CAN ID ───────────────────────────────────────── */
/* Sensor ECU → 판단 ECU (CAN FD, 23 bytes) */  /* ← 17 → 23 */
#define MSG_ID_ULTRASONIC         0x200U

/* 판단 ECU → 라즈베리파이 (CAN FD, 14 bytes) */  /* ← 9 → 14 */
#define MSG_ID_DISTANCE_LEVEL     0x400U

/* 판단 ECU → 라즈베리파이 (Classical, 1 byte) */
#define MSG_ID_EXIT_COMPLETE      0x401U

/* 판단 ECU → Motor ECU (Classical, 2 bytes) */
#define MSG_ID_VEHICLE_CTRL       0x100U

/* 라즈베리파이 → 판단 ECU (Classical, 6 bytes) */  /* ← 4 → 6 */
#define MSG_ID_VEHICLE_STATUS     0x201U

/* 라즈베리파이 → 판단 ECU (Classical, 1 byte) */
#define MSG_ID_AUTO_PARKING       0x300U

#endif
