/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef CUSTOM_TYPE_DEFS_H
#define CUSTOM_TYPE_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/**
 * @brief
 *  dbScan Clustering Configuration
 *
 * @details
 *  The structure contains the dbScan Clustering configuration used in data path
 *
 */
typedef struct DPU_dbScanCfg_t
{
    /*! @brief    enabled flag:  1-enabled 0-disabled */
    uint8_t      enabled;

    /*! @brief    maximum distance between adjacent points in cluster */
    float        epsilon;

    /*! @brief    minimum number of points to be deemed a cluster */
    uint8_t      min_points;

} DPU_dbScanCfg;

#ifdef __cplusplus
}
#endif

#endif 