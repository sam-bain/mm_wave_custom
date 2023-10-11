/* Copyright 2015 Gagarine Yaikhom (MIT License) */
#include "dbscan.h"

#define UNCLASSIFIED -1
#define NOISE -2

#define CORE_POINT 1
#define NOT_CORE_POINT 0

#define SUCCESS 0
#define FAILURE -3

typedef struct node_s node_t;
struct node_s {
    uint8_t index;
    node_t *next;
};

typedef struct epsilon_neighbours_s epsilon_neighbours_t;
struct epsilon_neighbours_s {
    uint8_t num_members;
    node_t *head, *tail;
};

node_t *create_node(uint8_t index);
int append_at_end(
     uint8_t index,
     epsilon_neighbours_t *en);
epsilon_neighbours_t *get_epsilon_neighbours(
    uint8_t index,
    point_t *points,
    uint8_t num_points,
    float epsilon);
// void print_epsilon_neighbours(
//     point_t *points,
//     epsilon_neighbours_t *en);
void destroy_epsilon_neighbours(epsilon_neighbours_t *en);
int expand(
    uint8_t index,
    uint8_t cluster_id,
    point_t *points,
    uint8_t num_points,
    float epsilon,
    uint8_t minpts);
int spread(
    uint8_t index,
    epsilon_neighbours_t *seeds,
    uint8_t cluster_id,
    point_t *points,
    uint8_t num_points,
    float epsilon,
    uint8_t minpts);
float euclidean_dist(point_t *a, point_t *b);
float adjacent_intensity_dist(point_t *a, point_t *b);
// uint8_t parse_input(
//     FILE *file,
//     point_t **points,
//     float *epsilon,
//     uint8_t *minpts);
// void print_points(
//     point_t *points,
//     uint8_t num_points);

node_t *create_node(uint8_t index)
{
    node_t *n = (node_t *) calloc(1, sizeof(node_t));
    if (n == NULL)
        perror("Failed to allocate node.");
    else {
        n->index = index;
        n->next = NULL;
    }
    return n;
}

int append_at_end(
     uint8_t index,
     epsilon_neighbours_t *en)
{
    node_t *n = create_node(index);
    if (n == NULL) {
        free(en);
        return FAILURE;
    }
    if (en->head == NULL) {
        en->head = n;
        en->tail = n;
    } else {
        en->tail->next = n;
        en->tail = n;
    }
    ++(en->num_members);
    return SUCCESS;
}

epsilon_neighbours_t *get_epsilon_neighbours(
    uint8_t index,
    point_t *points,
    uint8_t num_points,
    float epsilon)
{
    int i;
    epsilon_neighbours_t *en = (epsilon_neighbours_t *)
        calloc(1, sizeof(epsilon_neighbours_t));
    if (en == NULL) {
        perror("Failed to allocate epsilon neighbours.");
        return en;
    }
    for (i = 0; i < num_points; ++i) {
        if (i == index)
            continue;
        if (euclidean_dist(&points[index], &points[i]) > epsilon)
            continue;
        else {
            if (append_at_end(i, en) == FAILURE) {
                destroy_epsilon_neighbours(en);
                en = NULL;
                break;
            }
        }
    }
    return en;
}

// void print_epsilon_neighbours(
//     point_t *points,
//     epsilon_neighbours_t *en)
// {
//     if (en) {
//         node_t *h = en->head;
//         while (h) {
//             printf("(%lfm, %lf, %lf)\n",
//                    points[h->index].x,
//                    points[h->index].y,
//                    points[h->index].z);
//             h = h->next;
//         }
//     }
// }

void destroy_epsilon_neighbours(epsilon_neighbours_t *en)
{
    if (en) {
        node_t *t, *h = en->head;
        while (h) {
            t = h->next;
            free(h);
            h = t;
        }
        free(en);
    }
}

void dbscan(
    point_t *points,
    uint8_t num_points,
    float epsilon,
    uint8_t minpts)
{
    uint8_t i, cluster_id = 0;
    for (i = 0; i < num_points; ++i) {
        if (points[i].cluster_id == UNCLASSIFIED) {
            if (expand(i, cluster_id, points,
                       num_points, epsilon, minpts) == CORE_POINT)
                ++cluster_id;
        }
    }
}

int expand(
    uint8_t index,
    uint8_t cluster_id,
    point_t *points,
    uint8_t num_points,
    float epsilon,
    uint8_t minpts)
{
    int return_value = NOT_CORE_POINT;
    epsilon_neighbours_t *seeds =
        get_epsilon_neighbours(index, points,
                               num_points, epsilon);
    if (seeds == NULL)
        return FAILURE;

    if (seeds->num_members < minpts)
        points[index].cluster_id = NOISE;
    else {
        points[index].cluster_id = cluster_id;
        node_t *h = seeds->head;
        while (h) {
            points[h->index].cluster_id = cluster_id;
            h = h->next;
        }

        h = seeds->head;
        while (h) {
            spread(h->index, seeds, cluster_id, points,
                   num_points, epsilon, minpts);
            h = h->next;
        }

        return_value = CORE_POINT;
    }
    destroy_epsilon_neighbours(seeds);
    return return_value;
}

int spread(
    uint8_t index,
    epsilon_neighbours_t *seeds,
    uint8_t cluster_id,
    point_t *points,
    uint8_t num_points,
    float epsilon,
    uint8_t minpts)
{
    epsilon_neighbours_t *spread =
        get_epsilon_neighbours(index, points,
                       num_points, epsilon);
    if (spread == NULL)
        return FAILURE;
    if (spread->num_members >= minpts) {
        node_t *n = spread->head;
        point_t *d;
        while (n) {
            d = &points[n->index];
            if (d->cluster_id == NOISE ||
                d->cluster_id == UNCLASSIFIED) {
                if (d->cluster_id == UNCLASSIFIED) {
                    if (append_at_end(n->index, seeds)
                        == FAILURE) {
                        destroy_epsilon_neighbours(spread);
                        return FAILURE;
                    }
                }
                d->cluster_id = cluster_id;
            }
            n = n->next;
        }
    }

    destroy_epsilon_neighbours(spread);
    return SUCCESS;
}

float euclidean_dist(point_t *a, point_t *b)
{
    return sqrt(pow(a->x - b->x, 2) +
            pow(a->y - b->y, 2) +
            pow(a->z - b->z, 2));
}

// uint8_t parse_input(
//     FILE *file,
//     point_t **points,
//     float *epsilon,
//     uint8_t *minpts)
// {
//     uint8_t num_points, i = 0;
//     fscanf(file, "%f %i %i\n",
//            epsilon, minpts, &num_points);
//     point_t *p = (point_t *)
//         calloc(num_points, sizeof(point_t));
//     if (p == NULL) {
//         perror("Failed to allocate points array");
//         return 0;
//     }
//     while (i < num_points) {
//           fscanf(file, "%f %f %f\n",
//                  &(p[i].x), &(p[i].y), &(p[i].z));
//           p[i].cluster_id = UNCLASSIFIED;
//           ++i;
//     }
//     *points = p;
//     return num_points;
// }

// void print_points(
//     point_t *points,
//     uint8_t num_points)
// {
//     uint8_t i = 0;
//     printf("Number of points: %u\n"
//         " x     y     z     cluster_id\n"
//         "-----------------------------\n"
//         , num_points);
//     while (i < num_points) {
//           printf("%5.2f %5.2f %5.2f: %d\n",
//                  points[i].x,
//                  points[i].y, points[i].z,
//                  points[i].cluster_id);
//           ++i;
//     }
// }

// int main(void) {
//     point_t *points;
//     float epsilon;
//     uint8_t minpts;
//     uint8_t num_points =
//         parse_input(stdin, &points, &epsilon, &minpts);
//     if (num_points) {
//         dbscan(points, num_points, epsilon,
//                minpts, euclidean_dist);
//         printf("Epsilon: %lf\n", epsilon);
//         printf("Minimum points: %u\n", minpts);
//         print_points(points, num_points);
//     }
//     free(points);
//     return 0;
// }
