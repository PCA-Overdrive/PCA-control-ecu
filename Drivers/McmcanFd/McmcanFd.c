#include "McmcanFd.h"
#include "IfxCan_Can.h"
#include "IfxCan.h"
#include "IfxPort.h"
#include <string.h>

/* ================================================================
 * 모듈 핸들
 * ================================================================ */
IfxCan_Can       g_can;
IfxCan_Can_Node  g_canNode;

volatile uint32 g_dbg_rxMsgId   = 0U;
volatile uint32 g_dbg_rxCount   = 0U;
/* ================================================================
 * Message RAM 오프셋 (노드 0 기준)
 * ================================================================ */
#define CAN_NODE0_RAM_OFFSET   0x0U

/* ================================================================
 * RX 수신 플래그 & 버퍼
 * ================================================================ */
#ifdef NODE_JUDGMENT_ECU
static volatile boolean          s_rxUltrasonicReady    = FALSE;
static volatile boolean          s_rxVehicleStatusReady = FALSE;
static volatile boolean          s_rxAutoParkingReady   = FALSE;
static UltrasonicDistanceCmd_t   s_latestUltrasonic;
static VehicleStatusCmd_t        s_latestVehicleStatus;
static AutoParkingCmd_t          s_latestAutoParking;
#endif

#ifdef NODE_MOTOR_ECU
static volatile boolean          s_rxVehicleCtrlReady   = FALSE;
static VehicleControlCmd_t       s_latestVehicleCtrl;
#endif

/* ================================================================
 * McmcanFd_Init
 * ================================================================ */
void McmcanFd_Init(void)
{
    IfxPort_setPinModeOutput(&MODULE_P20, 6, IfxPort_OutputMode_pushPull,
                              IfxPort_OutputIdx_general);
    IfxPort_setPinLow(&MODULE_P20, 6);

    /* ── 모듈 초기화 ── */
    IfxCan_Can_Config canConfig;
    IfxCan_Can_initModuleConfig(&canConfig, &MODULE_CAN0);
    IfxCan_Can_initModule(&g_can, &canConfig);  /* rxf0n 설정 제거 */

    /* ── 노드 초기화 ── */
    IfxCan_Can_NodeConfig nodeCfg;
    IfxCan_Can_initNodeConfig(&nodeCfg, &g_can);

    nodeCfg.nodeId      = IfxCan_NodeId_0;
    nodeCfg.clockSource = IfxCan_ClockSource_both;

    nodeCfg.frame.type = IfxCan_FrameType_transmitAndReceive;
    nodeCfg.frame.mode = IfxCan_FrameMode_fdLongAndFast;

    nodeCfg.baudRate.baudrate      = MCMCAN_ARB_BAUDRATE;
    nodeCfg.baudRate.samplePoint   = 8000;
    nodeCfg.baudRate.syncJumpWidth = 1;

    nodeCfg.fastBaudRate.baudrate             = MCMCAN_DATA_BAUDRATE;
    nodeCfg.fastBaudRate.samplePoint          = 7500;
    nodeCfg.fastBaudRate.syncJumpWidth        = 1;
    nodeCfg.fastBaudRate.tranceiverDelayOffset = 0;

    nodeCfg.calculateBitTimingValues = TRUE;

    /* ── 핀 설정 ── */
    static const IfxCan_Can_Pins pins = {
        .txPin     = &IfxCan_TXD00_P20_8_OUT,
        .txPinMode = IfxPort_OutputMode_pushPull,
        .rxPin     = &IfxCan_RXD00B_P20_7_IN,
        .rxPinMode = IfxPort_InputMode_noPullDevice,
        .padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed2
    };
    nodeCfg.pins = &pins;

    /* ── TX 설정 ── */
    nodeCfg.txConfig.txMode                   = IfxCan_TxMode_dedicatedBuffers;
    nodeCfg.txConfig.dedicatedTxBuffersNumber  = 4U;
    nodeCfg.txConfig.txBufferDataFieldSize     = IfxCan_DataFieldSize_64;

    /* ── RX 설정 ── */
    nodeCfg.rxConfig.rxMode               = IfxCan_RxMode_fifo0;
    nodeCfg.rxConfig.rxFifo0Size          = 16U;
    nodeCfg.rxConfig.rxFifo0DataFieldSize = IfxCan_DataFieldSize_64;
    nodeCfg.rxConfig.rxFifo0OperatingMode = IfxCan_RxFifoMode_blocking;

    /* ── 인터럽트 설정 (노드 레벨) ── */
    nodeCfg.interruptConfig.rxFifo0NewMessageEnabled  = TRUE;
    nodeCfg.interruptConfig.rxf0n.priority            = ISR_PRIORITY_CAN_RX;
    nodeCfg.interruptConfig.rxf0n.typeOfService       = IfxSrc_Tos_cpu0;
    nodeCfg.interruptConfig.rxf0n.interruptLine       = IfxCan_InterruptLine_0;

    /* ── 필터 개수 ── */
#if defined(NODE_JUDGMENT_ECU)
    nodeCfg.filterConfig.standardListSize = 3U;
#elif defined(NODE_MOTOR_ECU)
    nodeCfg.filterConfig.standardListSize = 1U;
#else
    nodeCfg.filterConfig.standardListSize = 0U;
#endif

    /* ── Message RAM ── */
    nodeCfg.messageRAM.standardFilterListStartAddress = 0x000U;
    nodeCfg.messageRAM.rxBuffersStartAddress          = 0x100U;
    nodeCfg.messageRAM.rxFifo0StartAddress            = 0x200U;
    nodeCfg.messageRAM.txBuffersStartAddress          = 0x400U;

    IfxCan_Can_initNode(&g_canNode, &nodeCfg);

    /* ── 필터 설정 ── */
#ifdef NODE_JUDGMENT_ECU
    {
        IfxCan_Filter filter;
        filter.elementConfiguration = IfxCan_FilterElementConfiguration_storeInRxFifo0;
        filter.type                 = IfxCan_FilterType_classic;

        filter.number = 0;
        filter.id1 = MSG_ID_ULTRASONIC;
        filter.id2 = MSG_ID_ULTRASONIC;
        IfxCan_Can_setStandardFilter(&g_canNode, &filter);

        filter.number = 1;
        filter.id1 = MSG_ID_VEHICLE_STATUS;
        filter.id2 = MSG_ID_VEHICLE_STATUS;
        IfxCan_Can_setStandardFilter(&g_canNode, &filter);

        filter.number = 2;
        filter.id1 = MSG_ID_AUTO_PARKING;
        filter.id2 = MSG_ID_AUTO_PARKING;
        IfxCan_Can_setStandardFilter(&g_canNode, &filter);
    }
#endif

#ifdef NODE_MOTOR_ECU
    {
        IfxCan_Filter filter;
        filter.elementConfiguration = IfxCan_FilterElementConfiguration_storeInRxFifo0;
        filter.type                 = IfxCan_FilterType_classic;
        filter.number = 0;
        filter.id1 = MSG_ID_VEHICLE_CTRL;
        filter.id2 = MSG_ID_VEHICLE_CTRL;
        IfxCan_Can_setStandardFilter(&g_canNode, &filter);
    }
#endif
}

/* ================================================================
 * 내부 TX 헬퍼
 * data는 uint32 배열로 전달 (iLLD_1_20_0 API 방식)
 * ================================================================ */
static void sendFrame(uint32 msgId,
                      const void *payload,
                      uint8 payloadBytes,
                      boolean isFd)
{
    IfxCan_Message txMsg;
    IfxCan_Can_initMessage(&txMsg);

    txMsg.messageId          = msgId;
    txMsg.messageIdLength    = IfxCan_MessageIdLength_standard;
    txMsg.bufferNumber       = 0U;
    txMsg.storeInTxFifoQueue = FALSE;

    if (isFd) {
        txMsg.frameMode = IfxCan_FrameMode_fdLongAndFast;
    } else {
        txMsg.frameMode = IfxCan_FrameMode_standard;
    }

    if      (payloadBytes <= 8U)  txMsg.dataLengthCode = (IfxCan_DataLengthCode)payloadBytes;
    else if (payloadBytes <= 12U) txMsg.dataLengthCode = IfxCan_DataLengthCode_12;
    else if (payloadBytes <= 16U) txMsg.dataLengthCode = IfxCan_DataLengthCode_16;
    else if (payloadBytes <= 20U) txMsg.dataLengthCode = IfxCan_DataLengthCode_20;
    else                          txMsg.dataLengthCode = IfxCan_DataLengthCode_64;

    uint32 txData[16] = {0};
    memcpy(txData, payload, payloadBytes);

    /* 성공할 때까지 재시도, 단 횟수 제한 */
    uint32 retry = 0U;
    IfxCan_Status status;

    do {
        status = IfxCan_Can_sendMessage(&g_canNode, &txMsg, txData);
        retry++;
    } while (status == IfxCan_Status_notSentBusy && retry < 1000U);
}

/* ================================================================
 * Sensor ECU TX
 * ================================================================ */
#ifdef NODE_SENSOR_ECU
void McmcanFd_SendUltrasonic(const UltrasonicDistanceCmd_t *msg)
{
    /* 23 bytes → DLC 12 (CAN FD) */
    sendFrame(MSG_ID_ULTRASONIC, msg, sizeof(UltrasonicDistanceCmd_t), TRUE);
}
#endif

/* ================================================================
 * 판단 ECU TX
 * ================================================================ */
#ifdef NODE_JUDGMENT_ECU
void McmcanFd_SendDistanceLevel(const DistanceLevelCmd_t *msg)
{
    /* 14 bytes → DLC 10 (CAN FD) */
    sendFrame(MSG_ID_DISTANCE_LEVEL, msg, sizeof(DistanceLevelCmd_t), TRUE);
}

void McmcanFd_SendExitComplete(const ExitCompleteCmd_t *msg)
{
    /* 1 byte → Classical CAN */
    sendFrame(MSG_ID_EXIT_COMPLETE, msg, sizeof(ExitCompleteCmd_t), FALSE);
}

void McmcanFd_SendVehicleControl(const VehicleControlCmd_t *msg)
{
    /* 2 bytes → Classical CAN */
    sendFrame(MSG_ID_VEHICLE_CTRL, msg, sizeof(VehicleControlCmd_t), FALSE);
}
#endif

/* ================================================================
 * 판단 ECU RX
 * ================================================================ */
#ifdef NODE_JUDGMENT_ECU
boolean McmcanFd_RecvUltrasonic(UltrasonicDistanceCmd_t *out)
{
    if (!s_rxUltrasonicReady) return FALSE;
    memcpy(out, (const void *)&s_latestUltrasonic, sizeof(UltrasonicDistanceCmd_t));
    s_rxUltrasonicReady = FALSE;
    return TRUE;
}

boolean McmcanFd_RecvVehicleStatus(VehicleStatusCmd_t *out)
{
    if (!s_rxVehicleStatusReady) return FALSE;
    memcpy(out, (const void *)&s_latestVehicleStatus, sizeof(VehicleStatusCmd_t));
    s_rxVehicleStatusReady = FALSE;
    return TRUE;
}

boolean McmcanFd_RecvAutoParking(AutoParkingCmd_t *out)
{
    if (!s_rxAutoParkingReady) return FALSE;
    memcpy(out, (const void *)&s_latestAutoParking, sizeof(AutoParkingCmd_t));
    s_rxAutoParkingReady = FALSE;
    return TRUE;
}
#endif

/* ================================================================
 * Motor ECU RX
 * ================================================================ */
#ifdef NODE_MOTOR_ECU
boolean McmcanFd_RecvVehicleControl(VehicleControlCmd_t *out)
{
    if (!s_rxVehicleCtrlReady) return FALSE;
    memcpy(out, (const void *)&s_latestVehicleCtrl, sizeof(VehicleControlCmd_t));
    s_rxVehicleCtrlReady = FALSE;
    return TRUE;
}
#endif

/* ================================================================
 * RX 인터럽트 핸들러
 * ================================================================ */
IFX_INTERRUPT(McmcanFd_IsrRxHandler, 0, ISR_PRIORITY_CAN_RX)
{
    IfxCan_Message rxMsg;
    IfxCan_Can_initMessage(&rxMsg);
    rxMsg.readFromRxFifo0 = TRUE;

    uint32 rxData[16] = {0};

    IfxCan_Can_readMessage(&g_canNode, &rxMsg, rxData);

    g_dbg_rxMsgId = rxMsg.messageId;   /* ← 추가 */
    g_dbg_rxCount++;                    /* ← 추가 */

    switch (rxMsg.messageId)
    {
#ifdef NODE_JUDGMENT_ECU
        case MSG_ID_ULTRASONIC:
            memcpy((void *)&s_latestUltrasonic, rxData, sizeof(UltrasonicDistanceCmd_t));
            s_rxUltrasonicReady = TRUE;
            break;

        case MSG_ID_VEHICLE_STATUS:
            memcpy((void *)&s_latestVehicleStatus, rxData, sizeof(VehicleStatusCmd_t));
            s_rxVehicleStatusReady = TRUE;
            break;

        case MSG_ID_AUTO_PARKING:
            memcpy((void *)&s_latestAutoParking, rxData, sizeof(AutoParkingCmd_t));
            s_rxAutoParkingReady = TRUE;
            break;
#endif

#ifdef NODE_MOTOR_ECU
        case MSG_ID_VEHICLE_CTRL:
            memcpy((void *)&s_latestVehicleCtrl, rxData, sizeof(VehicleControlCmd_t));
            s_rxVehicleCtrlReady = TRUE;
            break;
#endif

        default:
            break;
    }

    /* ── RX FIFO0 인터럽트 플래그 클리어 ── */
    IfxCan_Node_clearInterruptFlag(g_canNode.node, IfxCan_Interrupt_rxFifo0NewMessage);
}
