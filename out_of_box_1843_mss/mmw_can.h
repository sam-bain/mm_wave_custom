#ifndef MMW_CAN_H
#define MMW_CAN_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ti/drivers/soc/soc.h>
#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>

void Can_Initialize(SOC_Handle socHandle);

void CAN_writeObjData(DPIF_PointCloudCartesian* objOut, uint32_t numObjOut);

void CAN_process1HzTasks(void);

void CAN_processTx(void);



#endif /*MMW_CAN_H*/
