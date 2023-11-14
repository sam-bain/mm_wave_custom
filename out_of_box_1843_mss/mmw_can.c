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

#include <ti/drivers/canfd/canfd.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>

#include "mmw_can.h"
// #include "dronecan_msgs.h"
#include "dsdlc_generated/include/dronecan_msgs.h"


/*********Libcanard Stuff*************/
#ifndef _GNU_SOURCE
# define _GNU_SOURCE
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

#define PROXIMITY_SENSOR_NODE_ID_BASE 30
/*
  hold our node status as a static variable. It will be updated on any errors
 */
static struct uavcan_protocol_NodeStatus node_status;
/*************************************/


# define M_PI           3.14159265358979323846 

#define CANARD_ENABLE_TAO 1 //Option to send length of obstacle list in canard CAN frame 

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

const float radar_angles[4] = {52.3, 0, -52.3, 0}; //The angle of each of the radars on the drone (CCW +ve, 0 = straight ahead). Order is defined by standard X4 motor order {FR, RL, FL, RR}
const float radar_pos[4][3] = {{0.271,  0.311, 0.18}, //The position of the front right radar on the drone {x, y, z} [m]
                               {    0,      0,    0}, //The position of the rear left radar on the drone {x, y, z} [m]
                               {0.271, -0.311, 0.18}, //The position of the front left radar on the drone {x, y, z} [m]
                               {    0,      0,    0}}; //The position of the rear right radar on the drone {x, y, z} [m]

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/
volatile uint32_t gTxDoneFlag = 0, gRxDoneFlag = 0, gParityErrFlag = 0;
volatile uint32_t gTxPkts = 0, gRxPkts = 0, gErrStatusInt = 0;
volatile uint32_t iterationCount = 0U;
uint32_t          dataLength     = 0U;
uint32_t          msgLstErrCnt   = 0U;
uint32_t          gDisplayStats  = 0;

rlOsiMutexHdl_t mutexHandle;
rlInt8_t mutexName[8] = "CAN_lock";



/**************************************************************************
 *************************** MCAN Global Definitions ***************************
 **************************************************************************/

uint8_t             rxData[64U];
uint32_t            txDataLength, rxDataLength;

/*Uncomment one of the below lines to choose CAN-FD or CAN classic*/
CANFD_MCANFrameType frameType = CANFD_MCANFrameType_CLASSIC;
// CANFD_MCANFrameType frameType = CANFD_MCANFrameType_FD;

static void MCANAppInitParams(CANFD_MCANInitParams *mcanCfgParams);

CANFD_Handle       canHandle;
CANFD_MsgObjHandle txMsgObjHandle;
CANFD_MsgObjHandle rxMsgObjHandle;

CANFD_MCANMsgObjCfgParams txMsgObjectParams;


static uint64_t timestamp_usec = 0;

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer);
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id);
static uint64_t micros64(void);

void Can_Initialize(SOC_Handle socHandle)
{

    int32_t errCode = 0;
    int32_t retVal  = 0;
    // int32_t j = 0;
    // CANFD_OptionTLV             optionTLV;
    CANFD_MCANInitParams      mcanCfgParams;
    CANFD_MCANBitTimingParams mcanBitTimingParams;
#if 0
    CANFD_MCANMsgObjectStats    msgObjStats;
    CANFD_MCANErrCntStatus      errCounter;
    CANFD_MCANProtocolStatus    protoStatus;
#endif
    CANFD_MCANMsgObjCfgParams rxMsgObjectParams;
    

    gTxDoneFlag = 0;
    gRxDoneFlag = 0;

    /* Setup the PINMUX to bring out the XWR18xx CAN pins */
    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PINE14_PADAE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PINE14_PADAE, SOC_XWR18XX_PINE14_PADAE_CANFD_TX);

    Pinmux_Set_OverrideCtrl(SOC_XWR18XX_PIND13_PADAD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR18XX_PIND13_PADAD, SOC_XWR18XX_PIND13_PADAD_CANFD_RX);


    /* Configure the divide value for MCAN source clock */
    SOC_setPeripheralClock(socHandle, SOC_MODULE_MCAN, SOC_CLKSOURCE_VCLK, 4U, &errCode);

    /* Initialize peripheral memory */
    SOC_initPeripheralRam(socHandle, SOC_MODULE_MCAN, &errCode);


    CSL_FINSR(0x43201450, 22, 22, 0x1U);
    CSL_FINSR(0x4320140C, 26, 16, 0x23U);


    MCANAppInitParams(&mcanCfgParams);

    /* Initialize the CANFD driver */
    canHandle = CANFD_init(0U, &mcanCfgParams, &errCode);
    if (canHandle == NULL)
    {
        CLI_write("Error: CANFD Module Initialization failed [Error code %d]\n", errCode);
        return;
    }

    /* Configuring 1Mbps and 5Mbps as nominal and data bit-rate respectively
        Prop seg: 8
        Ph seg 1: 6
        Ph Seg2 : 5
        Sync jump: 1
        BRP(Baud rate Prescaler): 2

        Nominal Bit rate = (40)/(((8+6+5)+1)*BRP) = 1Mhz

        Timing Params for Data Bit rate:
        Prop seg: 2
        Ph seg 1: 2
        Ph Seg2 : 3
        Sync jump: 1
        BRP(Baud rate Prescaler): 1

        Nominal Bit rate = (40)/(((2+2+3)+1)*BRP) = 5Mhz
    */
#if 1
    mcanBitTimingParams.nomBrp     = 0x2U;
    mcanBitTimingParams.nomPropSeg = 0x8U;
    mcanBitTimingParams.nomPseg1   = 0x6U;
    mcanBitTimingParams.nomPseg2   = 0x5U;
    mcanBitTimingParams.nomSjw     = 0x1U;
#else
    /*500Kbps NomBitRate: (40)/(((6+5+4)+1)*5)*/
    mcanBitTimingParams.nomBrp      = 0x5U;
    mcanBitTimingParams.nomPropSeg  = 0x6U;
    mcanBitTimingParams.nomPseg1    = 0x5U;
    mcanBitTimingParams.nomPseg2    = 0x4U;
    mcanBitTimingParams.nomSjw      = 0x1U;
#endif

#if 1 // 1 Mbps
    mcanBitTimingParams.dataBrp     = 0x8U;
    mcanBitTimingParams.dataPropSeg = 0x1U;
    mcanBitTimingParams.dataPseg1   = 0x2U;
    mcanBitTimingParams.dataPseg2   = 0x1U;
    mcanBitTimingParams.dataSjw     = 0x1U;
#elif 1 // 5 Mbps
    mcanBitTimingParams.dataBrp     = 0x1U;
    mcanBitTimingParams.dataPropSeg = 0x2U;
    mcanBitTimingParams.dataPseg1   = 0x2U;
    mcanBitTimingParams.dataPseg2   = 0x3U;
    mcanBitTimingParams.dataSjw     = 0x1U;
#else // 2 Mbps
    mcanBitTimingParams.dataBrp     = 0x4U;
    mcanBitTimingParams.dataPropSeg = 01U;
    mcanBitTimingParams.dataPseg1   = 0x2U;
    mcanBitTimingParams.dataPseg2   = 0x1U;
    mcanBitTimingParams.dataSjw     = 0x1U;

#endif
    /* Configure the CAN driver */
    retVal = CANFD_configBitTime(canHandle, &mcanBitTimingParams, &errCode);
    if (retVal < 0)
    {
        System_printf("Error: CANFD Module configure bit time failed [Error code %d]\n", errCode);
        return;
    }

    /* Setup the transmit message object */
    txMsgObjectParams.direction     = CANFD_Direction_TX;
    txMsgObjectParams.msgIdType     = CANFD_MCANXidType_29_BIT;
    txMsgObjectParams.msgIdentifier = 0xD1;


    txMsgObjHandle = CANFD_createMsgObject(canHandle, &txMsgObjectParams, &errCode);
    if (txMsgObjHandle == NULL)
    {
        System_printf("Error: CANFD create Tx message object failed [Error code %d]\n", errCode);
        return;
    }


    /* Setup the receive message object */
    rxMsgObjectParams.direction     = CANFD_Direction_RX;
    rxMsgObjectParams.msgIdType     = CANFD_MCANXidType_29_BIT;
    rxMsgObjectParams.msgIdentifier = 0xA1;

    rxMsgObjHandle = CANFD_createMsgObject(canHandle, &rxMsgObjectParams, &errCode);


}

void Canard_Initialize(uint8_t sensor_orientation) {
    /*
     Initializing the Libcanard instance.
     */
    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    canardSetLocalNodeID(&canard, PROXIMITY_SENSOR_NODE_ID_BASE + sensor_orientation);
}

/**
 *  @b Description
 *  @n
 *      Application implemented callback function to handle error and status interrupts.
 *
 *   @param[in] handle
 *      Handle to the CANFD Driver
 *  @param[in]  reason
 *      Cause of the interrupt which prompted the callback.
 *  @param[in]  errStatusResp
 *      Response structure populated with the value of the fields that caused the error or status interrupt.
 *      Processing of this structure is dependent on the callback reason.
 *
 *  @retval
 *      Not Applicable.
 */
static void MCANAppErrStatusCallback(CANFD_Handle handle, CANFD_Reason reason, CANFD_ErrStatusResp *errStatusResp)
{
    gErrStatusInt++;

    return;
}
/**
 *  @b Description
 *  @n
 *      Application implemented callback function to handle Tx complete and receive interrupts.
 *
 *   @param[in] handle
 *      Handle to the message object
 *   @param[in] reason
 *      Cause of the interrupt which prompted the callback.
 *
 *  @retval
 *      Not Applicable.
 */
static void MCANAppCallback(CANFD_MsgObjHandle handle, CANFD_Reason reason)
{    
    if (reason == CANFD_Reason_TX_COMPLETION)
    {
        {
            gTxPkts++;
            gTxDoneFlag = 1;
            return;
        }
    }
    if (reason == CANFD_Reason_RX)
    {
        {
            gRxPkts++;
            gRxDoneFlag = 1;
            return;
        }
    }
    if (reason == CANFD_Reason_TX_CANCELED)
    {
        {
            gTxPkts++;
            gTxDoneFlag = 1;
            gRxDoneFlag = 1;
            return;
        }
    }
}


static void MCANAppInitParams(CANFD_MCANInitParams *mcanCfgParams)
{
    /*Intialize MCAN Config Params*/
    memset(mcanCfgParams, sizeof(CANFD_MCANInitParams), 0);

    mcanCfgParams->fdMode                            = 0x0U;
    mcanCfgParams->brsEnable                         = 0x0U;
    mcanCfgParams->txpEnable                         = 0x0U;
    mcanCfgParams->efbi                              = 0x0U;
    mcanCfgParams->pxhddisable                       = 0x0U;
    mcanCfgParams->darEnable                         = 0x1U;
    mcanCfgParams->wkupReqEnable                     = 0x1U;
    mcanCfgParams->autoWkupEnable                    = 0x1U;
    mcanCfgParams->emulationEnable                   = 0x0U;
    mcanCfgParams->emulationFAck                     = 0x0U;
    mcanCfgParams->clkStopFAck                       = 0x0U;
    mcanCfgParams->wdcPreload                        = 0x0U;
    mcanCfgParams->tdcEnable                         = 0x1U;
    mcanCfgParams->tdcConfig.tdcf                    = 0U;
    mcanCfgParams->tdcConfig.tdco                    = 8U;
    mcanCfgParams->monEnable                         = 0x0U;
    mcanCfgParams->asmEnable                         = 0x0U;
    mcanCfgParams->tsPrescalar                       = 0x0U;
    mcanCfgParams->tsSelect                          = 0x0U;
    mcanCfgParams->timeoutSelect                     = CANFD_MCANTimeOutSelect_CONT;
    mcanCfgParams->timeoutPreload                    = 0x0U;
    mcanCfgParams->timeoutCntEnable                  = 0x0U;
    mcanCfgParams->filterConfig.rrfe                 = 0x1U;
    mcanCfgParams->filterConfig.rrfs                 = 0x1U;
    mcanCfgParams->filterConfig.anfe                 = 0x1U;
    mcanCfgParams->filterConfig.anfs                 = 0x1U;
    mcanCfgParams->msgRAMConfig.lss                  = 127U;
    mcanCfgParams->msgRAMConfig.lse                  = 64U;
    mcanCfgParams->msgRAMConfig.txBufNum             = 32U;
    mcanCfgParams->msgRAMConfig.txFIFOSize           = 0U;
    mcanCfgParams->msgRAMConfig.txBufMode            = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOSize      = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOWaterMark = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0size          = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0OpMode        = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0waterMark     = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO1size          = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1waterMark     = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1OpMode        = 64U;


    mcanCfgParams->eccConfig.enable        = 1;
    mcanCfgParams->eccConfig.enableChk     = 1;
    mcanCfgParams->eccConfig.enableRdModWr = 1;

    mcanCfgParams->errInterruptEnable  = 1U;
    mcanCfgParams->dataInterruptEnable = 1U;
    mcanCfgParams->appErrCallBack      = MCANAppErrStatusCallback;
    mcanCfgParams->appDataCallBack     = MCANAppCallback;
}

float wrap_360(const float angle)
{
    float res = fmodf(angle, 360.0f);
    if (res < 0) {
        res += 360.0f;
    }
    return res;
}

float * vectorMulti(float matrix[3][3], float array[3]) {
    /*Multiplies matrix by a vector to give a resultant vector*/
    int i;
    int k;
    float* result = (float*)malloc(3 * sizeof(float));

    for (i = 0; i < 3; ++i) {
        result[i] = 0;
        for (k = 0; k < 3; ++k) {
            result[i] += matrix[i][k] * array[k];
        }
    }

    return result;
}

float * vectorAdd(float vector1[3], const float vector2[3]) {
    /*Adds two vectors together to give a resultant vector*/
    int i;
    float* result = (float*)malloc(3 * sizeof(float));

    for (i = 0; i < 3; ++i) {
        result[i] = (vector1[i] + vector2[i]);
    }

    return result;
}

void coordinate_transform(DPIF_PointCloudCartesian* objPos, const uint8_t sensor_orientation)
{
    float theta =  M_PI / 180 * radar_angles[sensor_orientation]; //Radar angle in radians.

    float R_rotate[3][3] = {{-sin(theta), cos(theta),  0},
                            {cos(theta),  sin(theta),  0},     
                            {         0,           0, -1}};

    float obstacle_pos[3] = {objPos->x, objPos->y, objPos->z};   
    float* temp_array = vectorMulti(R_rotate, obstacle_pos);
    float* obstacle_local_pos = vectorAdd(temp_array, radar_pos[sensor_orientation]); 

    objPos->x = obstacle_local_pos[0];
    objPos->y = obstacle_local_pos[1];
    objPos->z = obstacle_local_pos[2];

    free(temp_array);
    free(obstacle_local_pos);
}

void populate_obstacle_message(DPIF_PointCloudCartesian* objPos, const uint8_t sensor_orientation, struct com_aeronavics_OBSTACLE* obstacle_message) 
{
    if (sensor_orientation != COM_AERONAVICS_PROXIMITYSENSOR_PROXIMITY_SENSOR_ID_UNDEFINED) {        
        coordinate_transform(objPos, sensor_orientation); 
    } 

    float xy;

    xy = sqrt(objPos->x*objPos->x + objPos->y*objPos->y);

    obstacle_message->distance = sqrt(xy*xy + objPos->z*objPos->z);
    obstacle_message->yaw = wrap_360(180/M_PI * atan2(objPos->y, objPos->x));
    obstacle_message->pitch = 180/M_PI * (M_PI/2 - atan2(xy, objPos->z));
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
void CAN_writeObjData(DPIF_PointCloudCartesian* objOut, DPIF_PointCloudSideInfo* objOutSideInfo, uint32_t numObjOut, const uint8_t sensor_orientation)
{
    timestamp_usec += 100000;

    uint8_t len;
    
    struct com_aeronavics_ProximitySensor* proximity_message = malloc(sizeof(struct com_aeronavics_ProximitySensor));

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;
    uint32_t message_len;

    // uint8_t* buffer = (uint8_t*) malloc(PROXIMITY_SENSOR_PROXIMITY_MAX_SIZE);
    uint8_t buffer[COM_AERONAVICS_PROXIMITYSENSOR_MAX_SIZE];

    proximity_message->sensor_id = sensor_orientation;

    for (len = 0; len < numObjOut; len++) {
        populate_obstacle_message(objOut+len, sensor_orientation, &proximity_message->obstacles.data[len]);    
    }   

    proximity_message->obstacles.len = len;

    message_len = com_aeronavics_ProximitySensor_encode(proximity_message, buffer);
    //Add proximity message to CAN queue
    canardBroadcast(&canard,
        COM_AERONAVICS_PROXIMITYSENSOR_SIGNATURE,
        COM_AERONAVICS_PROXIMITYSENSOR_ID,
        &transfer_id,
        CANARD_TRANSFER_PRIORITY_LOW,
        buffer,
        message_len); 

    free(proximity_message);

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

    rlRfDieIdCfg_t dieID = { 0 }; //Unique device ID which is embedded in each IWR1843 chip

    rlGetRfDieId(RL_DEVICE_MAP_INTERNAL_BSS, &dieID);

    id[0] = (dieID.lotNo & 0xff000000) >> 24;
    id[1] = (dieID.lotNo & 0x00ff0000) >> 16;
    id[2] = (dieID.lotNo & 0x0000ff00) >> 8;
    id[3] = dieID.lotNo & 0x000000ff;
    
    id[4] = (dieID.waferNo & 0xff000000) >> 24;
    id[5] = (dieID.waferNo & 0x00ff0000) >> 16;
    id[6] = (dieID.waferNo & 0x0000ff00) >> 8;
    id[7] = dieID.waferNo & 0x000000ff;
    
    id[8] = (dieID.devX & 0xff000000) >> 24;
    id[9] = (dieID.devX & 0x00ff0000) >> 16;
    id[10] = (dieID.devX & 0x0000ff00) >> 8;
    id[11] = dieID.devX & 0x000000ff;

    id[12] = (dieID.devY & 0xff000000) >> 24;
    id[13] = (dieID.devY & 0x00ff0000) >> 16;
    id[14] = (dieID.devY & 0x0000ff00) >> 8;
    id[15] = dieID.devY & 0x000000ff;

}

/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
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

    strncpy((char*)pkt.name.data, "RadarNode", sizeof(pkt.name.data));
    pkt.name.len = 9;
    // pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    int16_t status = canardRequestOrRespond(ins,
                                            transfer->source_node_id,
                                            UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                                            UAVCAN_PROTOCOL_GETNODEINFO_ID,
                                            &transfer->transfer_id,
                                            transfer->priority,
                                            CanardResponse,
                                            &buffer[0],
                                            total_size);
                                        
    CLI_write("Responding to request with status: %d\n", status);
}

/*
 This callback is invoked by the library when a new message or request or response is received.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    // switch on data type ID to pass to the right handler function
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
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
    if (transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
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
    int32_t           errCode = 0;
    // Transmitting
    const CanardCANFrame* txf;
    for (txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        // const int16_t tx_res = socketcanTransmit(socketcan, txf, 0);
        // const int16_t tx_res = Can_Transmit_Schedule(txf->id, txf->data, txf->data_len);
        int32_t transmit_status = CANFD_transmitData(txMsgObjHandle, txf->id, frameType, txf->data_len, txf->data, &errCode);
        canardPopTxQueue(&canard);
        Task_sleep(2);
    }
}

void CAN_processRx(void)
{
    uint32_t num_pkts;
    uint32_t pkt_index;

    int32_t             errCode, retVal;
    uint32_t            id;
    CANFD_MCANFrameType rxFrameType;
    CANFD_MCANXidType   rxIdType;

    CanardCANFrame rx_frame;

    // MMWave_osalMutexLock(mutexHandle, 10); 
    num_pkts = gRxPkts;
    gRxPkts = 0;
    // MMWave_osalMutexUnlock(mutexHandle);

    for (pkt_index = 0; pkt_index < num_pkts; pkt_index++) {
        /* Reset the receive buffer */
        memset(&rxData, 0, sizeof(rxData));
        dataLength = 0;

        retVal = CANFD_getData(rxMsgObjHandle, &id, &rxFrameType, &rxIdType, &rxDataLength, &rxData[0], &errCode);

        if (retVal < 0)
        {
            CLI_write("Error: CAN receive data for iteration %d failed [Error code %d]\n", iterationCount, errCode);
            return;
        }

        if (rxFrameType != frameType)
        {
            CLI_write("Error: CAN received incorrect frame type Sent %d Received %d for iteration %d failed\n", frameType, rxFrameType, iterationCount);
            return;
        }
        
        // Format for Libcanard and forward to canard module
        rx_frame.id = id | CANARD_CAN_FRAME_EFF;
        memcpy(&rx_frame.data, &rxData, CANARD_CAN_FRAME_MAX_DATA_LEN);
        rx_frame.data_len = rxDataLength;
        rx_frame.iface_id = 0;

        const uint64_t timestamp = micros64();

        int16_t recieve_status = canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
    
}
