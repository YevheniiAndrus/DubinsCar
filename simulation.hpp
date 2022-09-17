#pragma once

#include "car/car.hpp"
#include <opencv2/opencv.hpp>

using TangentLine = std::pair<Point, Point>;
using OuterTangentLines = std::pair<TangentLine, TangentLine>;

enum class CarCommands{
    CAR_RIGHT = 1,
    CAR_STRAIGHT = 2
};

class Simulation{
public:
    Simulation(const DubinsCar& start, const DubinsCar& goal);
    void run(cv::Mat& img);

private:
    // calculate outer tangent lines between two circles
    static OuterTangentLines calculateTangentLines(const double x1, const double y1,
                                    const double r1, const double x2, const double y2,
                                    const double r2);

    // length of arc of circle c (x, y) with radius r
    // between point (x1, y1) and (x2, y2)
    static double arcLength(const double x1, const double y1,
                            const double x2, const double y2,
                            const double cx, const double cy,
                            const double r,
                            double& theta);

    void updateCar(DubinsCar& car, double delta, CarCommands command);

private:
    DubinsCar m_start;
    DubinsCar m_goal;
};