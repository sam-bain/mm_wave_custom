/********************************************************************************************************************************
 *
 *  FILE:           mmw_can.c
 *
 *  SUB-SYSTEM:     Avionics
 *
 *  COMPONENT:      radar_OA
 *
 *  TARGET:         IWR1843
 *
 *  PLATFORM:
 *
 *  AUTHOR:         Sam Bain
 *
 *  DATE CREATED:   26-10-2023
 *
 *  DESCRIPTION:    Custom Aeronavics library for outputting obstacle data over CAN to Cube Flight Controller running Ardupilot
 *
 *
 *  Copyright 2018 Aeronavics Ltd
 *
 ********************************************************************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/utils/hsiheader/hsiheader.h>

#include <ti/drivers/can/can.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>

#include "mmw_can.h"
// #include "dronecan_msgs.h"
#include "dsdlc_generated/include/dronecan_msgs.h"

/*********Libcanard Stuff*************/
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <canard.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>

/*
  libcanard library instance and a memory pool for it to use
 */
static CanardInstance canard;
static uint8_t memory_pool[1024];

/*
  in this simple example we will use a fixed CAN node ID. This would
  need to be a parameter or use dynamic node allocation in a real
  application
 */
#define MY_NODE_ID 97

/*
  hold our node status as a static variable. It will be updated on any errors
 */
static struct uavcan_protocol_NodeStatus node_status;
/*************************************/

#define M_PI 3.14159265358979323846

typedef enum sensor_id_e
{
    PROXIMITY_SENSOR_ID_FRONT_RIGHT,
    PROXIMITY_SENSOR_ID_REAR_LEFT,
    PROXIMITY_SENSOR_ID_FRONT_LEFT,
    PROXIMITY_SENSOR_ID_REAR_RIGHT
} Sensor_id_type;

/*!
 * @brief
 *  Message types used in Millimeter Wave Demo for the communication between
 *  target and host, and also for Mailbox communication
 *  between MSS and DSS on the XWR16xx platform. Message types are used to indicate
 *  different type detection information sent out from the target.
 *
 */
typedef enum MmwDemo_output_message_type_e
{
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,

    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    MMWDEMO_OUTPUT_MSG_MAX
} MmwDemo_output_message_type;

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/** \brief DCAN input clock - 20MHz */
#define DCAN_APP_INPUT_CLK              (20000000U)
/** \brief DCAN output bit rate - 1MHz */
#define DCAN_APP_BIT_RATE               (1000000U)
/** \brief DCAN Propagation Delay - 700ns */
#define DCAN_APP_PROP_DELAY             (700U)
/** \brief DCAN Sampling Point - 70% */
#define DCAN_APP_SAMP_PT                (70U)

/** \brief DCAN TX message object used */
#define DCAN_TX_MSG_OBJ                 (0x1U)
/** \brief DCAN RX message object used */
#define DCAN_RX_MSG_OBJ                 (0x2U)

#define DCAN_MSG_OBJ_1                   (0x4)
#define DCAN_MSG_OBJ_2                   (0x6)

/** \brief Message Object Size*/
#define DCAN_MSG_OBJ_SIZE                (0x20U)

/** \brief DCAN Message Object RAM Address */
#define DCAN_MSG_OBJ_RAM_ADDR_1         ((SOC_XWR18XX_MSS_DCAN_MEM_BASE_ADDRESS) + \
                                         (DCAN_MSG_OBJ_1 * DCAN_MSG_OBJ_SIZE))
#define DCAN_MSG_OBJ_RAM_ADDR_2         ((SOC_XWR18XX_MSS_DCAN_MEM_BASE_ADDRESS) + \
                                        (DCAN_MSG_OBJ_2 * DCAN_MSG_OBJ_SIZE))

volatile uint32_t gTxDoneFlag = 0, gRxDoneFlag = 0, gParityErrFlag = 0;
uint32_t iterationCount = 0U;
volatile uint32_t gTxPkts = 0, gRxPkts = 0, gErrStatusInt = 0;
CAN_DCANCfgParams appDcanCfgParams;
CAN_DCANMsgObjCfgParams appDcanTxCfgParams;
CAN_DCANMsgObjCfgParams appDcanRxCfgParams;
CAN_DCANBitTimeParams appDcanBitTimeParams;
CAN_DCANData appDcanTxData;
CAN_DCANData appDcanRxData;
uint32_t dataLength = 0U;
uint32_t msgLstErrCnt = 0U;
uint32_t dataMissMatchErrCnt = 0U;
uint32_t gDisplayStats = 0;

static uint64_t timestamp_usec = 0;

CAN_Handle canHandle;
CAN_MsgObjHandle txMsgObjHandle;
CAN_MsgObjHandle rxMsgObjHandle;

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id);
static uint64_t micros64(void);

static void DCANAppInitParams(CAN_DCANCfgParams *dcanCfgParams,
                              CAN_DCANMsgObjCfgParams *dcanTxCfgParams,
                              CAN_DCANMsgObjCfgParams *dcanRxCfgParams,
                              CAN_DCANData *dcanTxData);

int32_t DCANAppCalcBitTimeParams(uint32_t clkFreq,
                                 uint32_t bitRate,
                                 uint32_t refSamplePnt,
                                 uint32_t propDelay,
                                 CAN_DCANBitTimeParams *bitTimeParams);

/**************************************************************************
*************************** CAN Driver Initialize Function ***********************
**************************************************************************/
void Can_Initialize(SOC_Handle socHandle)
{
    int32_t retVal = 0;
    int32_t errCode = 0;

    /* Setup the PINMUX to bring out the XWR18xx CAN pins */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINE15_PADAG, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINE15_PADAG, SOC_XWR18XX_PINE15_PADAG_CAN_TX);

    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINE13_PADAF, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINE13_PADAF, SOC_XWR18XX_PINE13_PADAF_CAN_RX);

    /* Configure the divide value for DCAN source clock */
    SOC_setPeripheralClock(socHandle, SOC_MODULE_DCAN, SOC_CLKSOURCE_VCLK, 9U, &errCode);
    /* Initialize peripheral memory */
    SOC_initPeripheralRam(socHandle, SOC_MODULE_DCAN, &errCode);
    /* Initialize the DCAN parameters that need to be specified by the application */
    DCANAppInitParams(&appDcanCfgParams, &appDcanTxCfgParams, &appDcanRxCfgParams, &appDcanTxData);

    /* Initialize the CAN driver */
    canHandle = CAN_init(&appDcanCfgParams, &errCode);

    if (canHandle == NULL)
    {
        CLI_write("Error: CAN Module Initialization failed [Error code %d]\n", errCode);
    }
    /* Set the desired bit rate based on input clock */
    retVal = DCANAppCalcBitTimeParams(DCAN_APP_INPUT_CLK / 1000000,
                                      DCAN_APP_BIT_RATE / 1000,
                                      DCAN_APP_SAMP_PT,
                                      DCAN_APP_PROP_DELAY,
                                      &appDcanBitTimeParams);
    if (retVal < 0)
    {
        CLI_write("Error: CAN Module bit time parameters are incorrect \n");
    }
    /* Configure the CAN driver */
    retVal = CAN_configBitTime(canHandle, &appDcanBitTimeParams, &errCode);
    if (retVal < 0)
    {
        CLI_write("Error: CAN Module configure bit time failed [Error code %d]\n", errCode);
    }
    /* Setup the transmit message object */
    txMsgObjHandle = CAN_createMsgObject(canHandle, DCAN_TX_MSG_OBJ, &appDcanTxCfgParams,
                                         &errCode);
    if (txMsgObjHandle == NULL)
    {
        CLI_write("Error: CAN create Tx message object failed [Error code %d]\n", errCode);
    }
    /* Setup the receive message object */
    rxMsgObjHandle = CAN_createMsgObject(canHandle, DCAN_RX_MSG_OBJ, &appDcanRxCfgParams,
                                         &errCode);
    if (rxMsgObjHandle == NULL)
    {
        CLI_write("Error: CAN create Rx message object failed [Error code %d]\n", errCode);
    }

    /*
    Initializing the Libcanard instance.
    */
    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    canardSetLocalNodeID(&canard, MY_NODE_ID);
}


/**************************************************************************
******************** CAN Bit Timing caluculation *****************
**************************************************************************/
int32_t DCANAppCalcBitTimeParams(uint32_t clkFreq,
                                 uint32_t bitRate,
                                 uint32_t refSamplePnt,
                                 uint32_t propDelay,
                                 CAN_DCANBitTimeParams *bitTimeParams)
{
    Double tBitRef = 1000 * 1000 / bitRate;
    Double newBaud = 0, newNProp = 0, newNSeg = 0, newSjw = 0, newP = 0;
    Double nQRef, nProp, fCan, nQ, nSeg, baud, sp, p, newSp = 0;
    float tQ;
    for (p = 1; p <= 1024; p++)
    {
        tQ = ((p / clkFreq) * 1000.0);
        nQRef = tBitRef / tQ;
        if ((nQRef >= 8) && (nQRef <= 25))
        {
            nProp = ceil(propDelay / tQ);
            fCan = clkFreq / p;
            nQ = fCan / bitRate * 1000;
            nSeg = ceil((nQ - nProp - 1) / 2);
            if ((nProp <= 8) && (nProp > 0) && (nSeg <= 8) && (nSeg > 0))
            {
                baud = fCan / (1 + nProp + 2 * nSeg) * 1000;
                sp = (1 + nProp + nSeg) / (1 + nProp + nSeg + nSeg) * 100;
                if ((abs(baud - bitRate)) < (abs(newBaud - bitRate)))
                {
                    newBaud = baud;
                    newNProp = nProp;
                    newNSeg = nSeg;
                    newSjw = (nSeg < 4) ? nSeg : 4;
                    newP = p - 1;
                    newSp = sp;
                }
                else if ((abs(baud - bitRate)) == (abs(newBaud - bitRate)))
                {
                    if ((abs(sp - refSamplePnt)) < (abs(newSp - refSamplePnt)))
                    {
                        newBaud = baud;
                        newNProp = nProp;
                        newNSeg = nSeg;
                        newSjw = (nSeg < 4) ? nSeg : 4;
                        newP = p - 1;
                        newSp = sp;
                    }
                }
            }
        }
    }
    if ((newBaud == 0) || (newBaud > 1000))
    {
        return -1;
    }
    bitTimeParams->baudRatePrescaler = (((uint32_t)newP) & 0x3F);
    bitTimeParams->baudRatePrescalerExt =
        ((((uint32_t)newP) & 0x3C0) ? (((uint32_t)newP) & 0x3C0) >> 6 : 0);
    bitTimeParams->syncJumpWidth = ((uint32_t)newSjw) - 1;
    /* propSeg = newNProp, phaseSeg = newNSeg, samplePoint = newSp
     * nominalBitTime = (1 + newNProp + 2 * newNSeg), nominalBitRate = newBaud
     * brpFreq = clkFreq / (brp + 1), brpeFreq = clkFreq / (newP + 1)
     * brp = bitTimeParams->baudRatePrescaler;
     */
    bitTimeParams->timeSegment1 = newNProp + newNSeg - 1;
    bitTimeParams->timeSegment2 = newNSeg - 1;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Application implemented callback function to handle error and status interrupts.
 *
 *   @param[in] handle
 *      Handle to the CAN Driver
 *   @param[in] errStatusResp
 *      Response structure containing the Error and status information
 *
 *  @retval
 *      Not Applicable.
 */
static void DCANAppErrStatusCallback(CAN_Handle handle, CAN_ErrStatusResp *errStatusResp)
{
    gErrStatusInt++;
    if (errStatusResp->parityError == 1)
    {
        gParityErrFlag = 1;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Application implemented callback function to handle Tx complete and receive interrupts.
 *
 *   @param[in] handle
 *      Handle to the message object
 *   @param[in] msgObjectNum
 *      Message object number
 *   @param[in] direction
 *      Direction of the object number
 *
 *  @retval
 *      Not Applicable.
 */
static void DCANAppCallback(CAN_MsgObjHandle handle, uint32_t msgObjectNum, CAN_Direction direction)
{
    int32_t errCode, retVal;

    CanardCANFrame rx_frame;

    if (direction == CAN_Direction_TX)
    {
        if (msgObjectNum != DCAN_TX_MSG_OBJ)
        {
            System_printf("Error: Tx callback received for incorrect Message Object %d\n",
                          msgObjectNum);
            return;
        }
        else
        {
            gTxPkts++;
            gTxDoneFlag = 1;
            return;
        }
    }
    if (direction == CAN_Direction_RX)
    {
        CLI_write("CAN message recieved\n");

        if (msgObjectNum != DCAN_RX_MSG_OBJ)
        {
            CLI_write("Error: Rx callback received for incorrect Message Object %d\n",
                          msgObjectNum);
            return;
        }
        else
        {
            /* Reset the receive buffer */
            memset(&appDcanRxData, 0, sizeof(appDcanRxData));
            dataLength = 0;
            retVal = CAN_getData(handle, &appDcanRxData, &errCode);
            if (retVal < 0)
            {
                CLI_write ("Error: CAN receive data for iteration %d failed [Error code %d]\n", iterationCount, errCode);
                return;
            }
            /* Check if sent data is lost or not */
            if (appDcanRxData.msgLostFlag == 1)
            {
                msgLstErrCnt++;
            }
            while (dataLength < appDcanRxData.dataLength)
            {
                if (appDcanRxData.msgData[dataLength] != appDcanTxData.msgData[dataLength])
                {
                    dataMissMatchErrCnt++;
                    CLI_write ("Error: CAN receive data mismatch for iteration %d at byte %d\n", iterationCount, dataLength);
                }
                dataLength++;
            }

            // Format for Libcanard and forward to canard module
            // rx_frame.id = id;
            // CLI_write("Recieved frame ID: %d\n", id);
            // uint32_t index;
            // CLI_write("Data: ");
            // for (index = 0; index < dataLength; index++)
            // {
            //     CLI_write("%d ", rxData[index]);
            // }

            // memcpy(&rx_frame.data, &rxData, CANARD_CAN_FRAME_MAX_DATA_LEN);
            // rx_frame.data_len = rxDataLength;

            // const uint64_t timestamp = micros64();

            // canardHandleRxFrame(&canard, &rx_frame, timestamp);

            gRxPkts++;
            gRxDoneFlag = 1;
            return;
        }
    }
}

/**************************************************************************
******************** CAN Parameters initialize Function *****************
**************************************************************************/
static void DCANAppInitParams(CAN_DCANCfgParams *dcanCfgParams,
                              CAN_DCANMsgObjCfgParams *dcanTxCfgParams,
                              CAN_DCANMsgObjCfgParams *dcanRxCfgParams,
                              CAN_DCANData *dcanTxData)
{
    /*Intialize DCAN Config Params*/
    dcanCfgParams->parityEnable = 0;
    dcanCfgParams->intrLine0Enable = 1;
    dcanCfgParams->intrLine1Enable = 1;
    dcanCfgParams->testModeEnable = 0;
    dcanCfgParams->eccModeEnable = 0;
    dcanCfgParams->stsChangeIntrEnable = 0;
    dcanCfgParams->autoRetransmitDisable = 1;
    dcanCfgParams->autoBusOnEnable = 0;
    dcanCfgParams->errIntrEnable = 1;
    dcanCfgParams->autoBusOnTimerVal = 0;
    dcanCfgParams->if1DmaEnable = 0;
    dcanCfgParams->if2DmaEnable = 0;
    dcanCfgParams->if3DmaEnable = 0;
    dcanCfgParams->ramAccessEnable = 0;
    dcanCfgParams->appCallBack = DCANAppErrStatusCallback;
    /*Intialize DCAN tx Config Params*/
    dcanTxCfgParams->xIdFlagMask = 0x1;
    dcanTxCfgParams->dirMask = 0x1;
    dcanTxCfgParams->msgIdentifierMask = 0x1FFFFFFF;
    dcanTxCfgParams->msgValid = 1;
    dcanTxCfgParams->xIdFlag = CAN_DCANXidType_29_BIT;
    dcanTxCfgParams->direction = CAN_Direction_TX;
    dcanTxCfgParams->msgIdentifier = 0xC1;
    dcanTxCfgParams->uMaskUsed = 1;
    dcanTxCfgParams->intEnable = 1;
    dcanTxCfgParams->remoteEnable = 0;
    dcanTxCfgParams->fifoEOBFlag = 1;
    dcanTxCfgParams->appCallBack = DCANAppCallback;
    /*Intialize DCAN Rx Config Params*/
    dcanRxCfgParams->xIdFlagMask = 0x1;
    dcanRxCfgParams->msgIdentifierMask = 0x1FFFFFFF;
    dcanRxCfgParams->dirMask = 0x1;
    dcanRxCfgParams->msgValid = 1;
    dcanRxCfgParams->xIdFlag = CAN_DCANXidType_29_BIT;
    dcanRxCfgParams->direction = CAN_Direction_RX;
    dcanRxCfgParams->msgIdentifier = 0xC1;
    dcanRxCfgParams->uMaskUsed = 1;
    dcanRxCfgParams->intEnable = 1;
    dcanRxCfgParams->remoteEnable = 0;
    dcanRxCfgParams->fifoEOBFlag = 1;
    dcanRxCfgParams->appCallBack = DCANAppCallback;
    /*Intialize DCAN Tx transfer Params*/
    dcanTxData->dataLength = DCAN_MAX_MSG_LENGTH;
    dcanTxData->msgData[0] = 0xA5;
    dcanTxData->msgData[1] = 0x5A;
    dcanTxData->msgData[2] = 0xFF;
    dcanTxData->msgData[3] = 0xFF;
    dcanTxData->msgData[4] = 0xC3;
    dcanTxData->msgData[5] = 0x3C;
    dcanTxData->msgData[6] = 0xB4;
    dcanTxData->msgData[7] = 0x4B;
}

float wrap_360(const float angle)
{
    float res = fmodf(angle, 360.0f);
    if (res < 0)
    {
        res += 360.0f;
    }
    return res;
}

void populate_proximity_message(DPIF_PointCloudCartesian *objPos, struct proximity_sensor_Proximity *proximity_message)
{

    float xy;

    xy = sqrt(objPos->x * objPos->x + objPos->y * objPos->y);

    proximity_message->distance = sqrt(xy * xy + objPos->z * objPos->z);
    proximity_message->yaw = wrap_360(180 / M_PI * atan2(objPos->x, objPos->y));
    proximity_message->pitch = 180 / M_PI * (M_PI / 2 - atan2(xy, objPos->z));

    // proximity_message->distance = 1;
    // proximity_message->yaw = 2;
    // proximity_message->pitch = 3;
}

/**
 *  @b Description
 *  @n
 *      Simple function to write obstacle data to Libcanard Queue
 *
 *  @param[in]  result obstacle data.
 *
 *  @retval
 *      Not Applicable.
 */
void CAN_writeObjData(DPIF_PointCloudCartesian *objOut, uint32_t numObjOut)
{
    int index;
    struct proximity_sensor_Proximity *proximity_message = malloc(sizeof(struct proximity_sensor_Proximity));

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;
    uint32_t len;

    // uint8_t* buffer = (uint8_t*) malloc(PROXIMITY_SENSOR_PROXIMITY_MAX_SIZE);
    uint8_t buffer[PROXIMITY_SENSOR_PROXIMITY_MAX_SIZE];

    if (numObjOut > 0)
    {
        proximity_message->sensor_id = PROXIMITY_SENSOR_ID_FRONT_LEFT;
        proximity_message->reading_type = PROXIMITY_SENSOR_PROXIMITY_READING_TYPE_GOOD;
        proximity_message->flags = 0;

        // Send single object per timestep for debug
        populate_proximity_message(objOut, proximity_message);
        len = proximity_sensor_Proximity_encode(proximity_message, buffer);

        // Add proximity message to CAN queue
        canardBroadcast(&canard,
                        PROXIMITY_SENSOR_PROXIMITY_SIGNATURE,
                        PROXIMITY_SENSOR_PROXIMITY_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        len);

        // for (index = 0; index < numObjOut; index++) {
        //     populate_proximity_message(objOut+index, proximity_message);
        //     len = proximity_sensor_Proximity_encode(proximity_message, buffer);

        //     //Add proximity message to CAN queue
        //     canardBroadcast(&canard,
        //         PROXIMITY_SENSOR_PROXIMITY_SIGNATURE,
        //         PROXIMITY_SENSOR_PROXIMITY_ID,
        //         &transfer_id,
        //         CANARD_TRANSFER_PRIORITY_LOW,
        //         buffer,
        //         len);
        // }
    }

    free(proximity_message);
    // free(buffer);
}

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific
 */
static uint64_t micros64(void)
{
    // static uint64_t first_us;
    // struct timespec ts;
    // clock_gettime(CLOCK_MONOTONIC, &ts);
    // uint64_t tus = (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
    // if (first_us == 0) {
    //     first_us = tus;
    // }
    // return tus - first_us;
    return timestamp_usec;
}

/*
  get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
 */
static void getUniqueID(uint8_t id[16])
{
    // memset(id, 0, 16);
    // FILE *f = fopen("/etc/machine-id", "r");
    // if (f) {
    //     fread(id, 1, 16, f);
    //     fclose(f);
    // }
    int index;
    for (index = 0; index < 15; index++)
    {
        id[index] = 0;
    }
    id[15] = 30;
}

/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    CLI_write("GetNodeInfo request from %d\n", transfer->source_node_id);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = 1;
    pkt.software_version.minor = 2;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char *)pkt.name.data, "RadarNode", sizeof(pkt.name.data));
    pkt.name.len = 10;
    // pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
 This callback is invoked by the library when a new message or request or response is received.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    // switch on data type ID to pass to the right handler function
    if (transfer->transfer_type == CanardTransferTypeRequest)
    {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            handle_GetNodeInfo(ins, transfer);
            break;
        }
        }
    }
}

/*
 This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 by the local node.
 If the callback returns true, the library will receive the transfer.
 If the callback returns false, the library will ignore the transfer.
 All transfers that are addressed to other nodes are always ignored.

 This function must fill in the out_data_type_signature to be the signature of the message.
 */
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest)
    {
        // check if we want to handle a specific service request
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        }
    }
    // we don't want any other messages
    return false;
}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
static void send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = micros64() / 1000000ULL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
void CAN_process1HzTasks(void)
{
    timestamp_usec += 1000000;
    uint64_t timestamp_usec = micros64();
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
     Transmit the node status message
     */
    send_NodeStatus();
}

/**
 *  @b Description
 *  @n
 *      Transmits all frames from the TX queue
 *  @retval
 *      Not Applicable.
 */
void CAN_processTx(void)
{
    int32_t errCode = 0;
    int32_t retval = 0;
    int32_t i;
    // Transmitting
    const CanardCANFrame *txf;
    for (txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
        appDcanTxData.dataLength = txf->data_len;
        memcpy(&appDcanTxData.msgData, &txf->data, CANARD_CAN_FRAME_MAX_DATA_LEN);
        appDcanTxCfgParams.msgIdentifier = txf->id;
        retval = CAN_transmitData(txMsgObjHandle, &appDcanTxData, &errCode);
        if (retval < 0) {
            CLI_write("CAN transmit error, code: %d\n", errCode);
        } else {
            CLI_write("Transmitted CAN message:");
            for (i = 0; i < CANARD_CAN_FRAME_MAX_DATA_LEN; i++) {
                CLI_write(" %d", appDcanTxData.msgData[i]);
            }
            CLI_write("\n");
        }
        canardPopTxQueue(&canard);
    }
}
