#ifndef VISUAL_LOCATION_H
#define VISUAL_LOCATION_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef struct Landmark Landmark;
struct Landmark
{
    int id;
    Vec3f leftTop;
    Vec3f rightTop;
    Vec3f leftBottom;
    vector<Vec3f> circles;
};

typedef struct VisualData VisualData;
struct VisualData
{
    int landmark_id;
    double visual_theta;
    float visual_x;
    float visual_y;
    float d_theta;
};

void loadVisualParams();
VisualData getVisualLocalizeData(Mat srcImage);
Vec3f getPointAffinedPos(Vec3f& src, const Point center, double angle);


#endif 