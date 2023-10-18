#ifndef DBSCAN_H
#define DBSCAN_H

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct point_s point_t;
struct point_s {
    float x, y, z;
    int8_t cluster_id;
    float snr;
};

void dbscan(
    point_t *points,
    uint8_t num_points,
    float epsilon,
    uint8_t minpts,
    float override_threshold);

#endif /*DBSCAN_H*/
