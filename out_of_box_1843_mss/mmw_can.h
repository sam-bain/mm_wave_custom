#ifndef MMW_CAN_H
#define MMW_CAN_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ti/drivers/soc/soc.h>
#include "objectdetection.h"

void Can_Initialize(SOC_Handle socHandle);

void Canard_Initialize(uint8_t sensor_orientation);

void CAN_writeObjData(DPIF_PointCloudCartesian* objOut, DPIF_PointCloudSideInfo* objOutSideInfo, uint32_t numObjOut, const uint8_t sensor_orientation);

void CAN_process1HzTasks(void);

void CAN_processTx(void);

void CAN_processRx(void);



#endif /*MMW_CAN_H*/
