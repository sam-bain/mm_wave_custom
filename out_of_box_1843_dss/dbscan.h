#ifndef DBSCAN_H
#define DBSCAN_H

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// ! @brief Configuration set in config file for dbScan algorithm
typedef struct DPU_dbScanCfg_t 
{
    /*! @brief    enabled flag:  1-enabled 0-disabled */
    uint8_t      enabled;

    //! @brief the number of points required to be defined as a cluster 
    uint8_t min_points;

    //! @brief the maximum distance between points in a cluster [m]
    float epsilon;

    //! @brief points above this intensity [dB] will not be classed as noise regardless of whether they are in an cluster or not
    int16_t override_intensity;

    //! @brief points beyond this distance [m] will not be classed as noise regardless of whether they are in an cluster or not
    int16_t override_distance;

} DPU_dbScanCfg;

typedef struct point_s point_t;
struct point_s {
    float x, y, z;
    int8_t cluster_id;
    float snr;
};

void dbscan(
    point_t *points,
    uint8_t num_points,
    DPU_dbScanCfg dbScanCfg);

#endif /*DBSCAN_H*/
