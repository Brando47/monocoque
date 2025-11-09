#ifndef _E36CLUSTERDATA_H
#define _E36CLUSTERDATA_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    char start_byte;
    uint32_t rpms;
    uint32_t velocity;
    double wheelspeed;
    uint32_t gas;
    double fuel;
    uint32_t tyretemp;
    double turboboost;
}
E36ClusterData;


#endif
