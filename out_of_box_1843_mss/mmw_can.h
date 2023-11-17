#ifndef MMW_CAN_H
#define MMW_CAN_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ti/drivers/soc/soc.h>
#include "objectdetection.h"

#define USE_CUSTOM_MESSAGE_TYPE 0 

#define USE_ADC_FOR_ORIENTATION 0

#if !(USE_ADC_FOR_ORIENTATION)
    #define RIGHT_RADAR_UNIQUE_ID 0x17
    #define LEFT_RADAR_UNIQUE_ID 0x26
#endif

void Can_Initialize(SOC_Handle socHandle);

void CAN_setSensorOrientation(uint8_t orientation);

void Canard_Initialize();

void CAN_writeCustomObjData(DPIF_PointCloudCartesian* objOut, DPIF_PointCloudSideInfo* objOutSideInfo, uint32_t numObjOut);
void CAN_writeStandardObjData(DPIF_PointCloudCartesian* objOut, DPIF_PointCloudSideInfo* objOutSideInfo, uint32_t numObjOut);

# if (USE_CUSTOM_MESSAGE_TYPE)
    #define CAN_writeObjData CAN_writeCustomObjData
# else
    #define CAN_writeObjData CAN_writeStandardObjData
# endif

void CAN_process1HzTasks(void);

void CAN_processTx(void);

void CAN_processRx(void);



#endif /*MMW_CAN_H*/
