#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <utility>
#include <functional>
#include "car/car.hpp"
#include "simulation.hpp"

using outer_tangent_line = std::pair<cv::Point, cv::Point>;

void tangentLine(
    cv::Mat img,
    const cv::Point& p1, int r1,
    const cv::Point& p2, int r2){

    // math taken from here - https://math.stackexchange.com/questions/719758/inner-tangent-between-two-circles-formula
    // treat the bigger circle with p1 center
    // and smaller circle with p2 center
    // create new circle with r3 = r1 - r2 radius
    // inside the bigger p1 circle

    // distance between p1 and p2 circles
    double r3 = static_cast<double>(r1 - r2);
    double hyp = sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    double tangent_lenth = sqrt(hyp * hyp - r3 * r3);

    double phi = std::atan2(p2.y - p1.y, p2.x - p1.x) + std::acos(r3 / hyp);

    auto calc_tang_pair = [](cv::Point p1, cv::Point p2, int r1, int r2, double phi){
        // first circle tangent point
        double t1x = static_cast<double>(p1.x) + static_cast<double>(r1) * std::cos(phi);
        double t1y = static_cast<double>(p1.y) + static_cast<double>(r1) * std::sin(phi);
        cv::Point pt1(static_cast<int>(t1x), static_cast<int>(t1y));

        // second circle tangent point
        double t2x = static_cast<double>(p2.x) + static_cast<double>(r2) * std::cos(phi);
        double t2y = static_cast<double>(p2.y) + static_cast<double>(r2) * std::sin(phi);
        cv::Point pt2(static_cast<int>(t2x), static_cast<int>(t2y));
        return std::make_pair(pt1, pt2);
    };

    const auto& first_pair = calc_tang_pair(p1, p2, r1, r2, phi);
    cv::line(img, first_pair.first, first_pair.second, cv::Scalar(0, 0, 255), 1);

    // calculate another pair of tangent points
    phi = std::atan2(p2.y - p1.y, p2.x - p1.x) - std::acos(r3 / hyp);
    const auto& second_pair = calc_tang_pair(p1, p2, r1, r2, phi);
    cv::line(img, second_pair.first, second_pair.second, cv::Scalar(0, 0, 255), 1);
}

int main(){
    int img_width = 1024;
    int img_height = 768;
    cv::Mat blank = cv::Mat(cv::Size(img_width, img_height), CV_8UC3, cv::Scalar(255, 255, 255));
    
    DubinsCar start(cv::Point(200, 250), 40, M_PI / 10);
    DubinsCar goal(cv::Point(660, 440), 40, -M_PI / 2);    
    Simulation sim(start, goal);
    sim.run(blank);

    cv::imshow("test", blank);
    cv::waitKey(0);
    return 0;
}