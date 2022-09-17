#pragma once

#include <cmath>

// This is need to make calculation works without dependencies on OpenCV stucture
// OpenCV only need to make rendering
struct Point{
    double x{0.0};
    double y{0.0};

    Point() = default;
    Point(double x, double y){
        this->x = x;
        this->y = y;
    }
};

// tangent circel to car's direction
// with radius equal to car's turning radius

//      |  _ _ _
//      | /     \ 
//      |/       \
//      |    C   /
//      |\      /
//      | \____/
struct AlignedCircle{
    Point center;
};

struct DubinsCar{
    Point position;
    double direction; // angle between heading car and X-axis
    double arc_theta; // angle between vector aligned circle center to car starting position and X-axis
    double radius;
    AlignedCircle aligned_circle;

    explicit DubinsCar(const Point& pos, double r, double dir) : 
        position(pos), radius(r), direction(dir) {}
};