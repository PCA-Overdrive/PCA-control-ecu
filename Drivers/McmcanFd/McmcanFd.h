#ifndef MCMCANFD_H
#define MCMCANFD_H

#include "Ifx_Types.h"
#include "IfxCan_Can.h"
#include "McmcanFd_Cfg.h"
#include "CanMsg.h"

extern IfxCan_Can       g_can;
extern IfxCan_Can_Node  g_canNode;

/* 공통 초기화 */
void McmcanFd_Init(void);

/* ── Sensor ECU TX ───────────────────────────────── */
#ifdef NODE_SENSOR_ECU
void McmcanFd_SendUltrasonic(const UltrasonicDistanceCmd_t *msg);
#endif

/* ── 판단 ECU TX ─────────────────────────────────── */
#ifdef NODE_JUDGMENT_ECU
void McmcanFd_SendDistanceLevel(const DistanceLevelCmd_t *msg);
void McmcanFd_SendExitComplete(const ExitCompleteCmd_t *msg);
void McmcanFd_SendVehicleControl(const VehicleControlCmd_t *msg);
#endif

/* ── 판단 ECU RX ─────────────────────────────────── */
#ifdef NODE_JUDGMENT_ECU
boolean McmcanFd_RecvUltrasonic(UltrasonicDistanceCmd_t *out);
boolean McmcanFd_RecvVehicleStatus(VehicleStatusCmd_t *out);
boolean McmcanFd_RecvAutoParking(AutoParkingCmd_t *out);
#endif

/* ── Motor ECU RX ────────────────────────────────── */
#ifdef NODE_MOTOR_ECU
boolean McmcanFd_RecvVehicleControl(VehicleControlCmd_t *out);
#endif

/* ISR */
void McmcanFd_IsrRxHandler(void);

#endif
