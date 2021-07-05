#ifndef UWB_LOCATION_H
#define UWB_LOCATION_H

#include <iostream>
#include <math.h>
#include <vector>
using namespace std;

extern int ANCHOR_NUM;
extern int ANCHOR_DIS_START;
extern int MAX_BUFF_SIZE;
static const double ZERO = 1e-9;

typedef struct vec2d vec2d;
struct vec2d
{
    double x;
    double y;
};

typedef struct Lcircle Lcircle;
struct Lcircle
{
    double x;
    double y;
    double r;
};

void loadUWBParams();
vec2d trilateration(const int *ids, const int *radius);
vec2d selectPoint(const vector<vec2d> points,const Lcircle circle);
bool isOutsideCircle(const vec2d point, const Lcircle circle);
vector<vec2d> insect(const Lcircle circle1, const Lcircle circle2);
vec2d optimizeByRatio(const vector<vec2d> points);

#endif