#ifndef DBSCAN_H
#define DBSCAN_H

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct point_s point_t;
struct point_s {
    double x, y, z;
    int cluster_id;
};

void dbscan(
    point_t *points,
    unsigned int num_points,
    double epsilon,
    unsigned int minpts);

#endif /*DBSCAN_H*/