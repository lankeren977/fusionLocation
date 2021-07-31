#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <math.h>
using namespace std;

typedef struct vec3d vec3d;
struct vec3d
{
    double x;
    double y;
    double theta;
};

vec3d fusion();

#endif