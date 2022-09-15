#pragma once

#include "car/car.hpp"
#include <opencv2/opencv.hpp>

using TangentLine = std::pair<cv::Point, cv::Point>;
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
    static OuterTangentLines calculateTangentLines(const DubinsCar& car1,
                                              const DubinsCar& car2);
    static double arcLength(DubinsCar& car, const cv::Point& tangent_point);

    void updateCar(DubinsCar& car, double delta, CarCommands command);

private:
    DubinsCar m_start;
    DubinsCar m_goal;
};