#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>

// tangent circel to car's direction
// with radius equal to car's turning radius

//      |  _ _ _
//      | /     \ 
//      |/       \
//      |    C   /
//      |\      /
//      | \____/
struct AlignedCircle{
    cv::Point center;
};

struct DubinsCar{
    cv::Point position;
    double direction; // angle between heading car and X-axis
    double arc_theta; // angle between vector aligned circle center to car starting position and X-axis
    double radius;
    AlignedCircle aligned_circle;

    explicit DubinsCar(const cv::Point& pos, double r, double dir) : 
        position(pos), radius(r), direction(dir) {}
};