#include "simulation.hpp"

Simulation::Simulation(
    const DubinsCar& start,
    const DubinsCar& goal) :
    m_start(start), 
    m_goal(goal) {}

OuterTangentLines Simulation::calculateTangentLines(
    const double x1,
    const double y1,
    const double r1,
    const double x2,
    const double y2,
    const double r2
){
    // lambda to calculate tangent point given Dubins car and angle
    // between vector start to goal position and tangent point
    auto tangent_point = [](const double x, const double y, const double r, const double phi){
        Point tangent;
        tangent.x = x + r * std::cos(phi);
        tangent.y = y + r * std::sin(phi);
        return tangent;
    };

    // math taken from here - https://math.stackexchange.com/questions/719758/inner-tangent-between-two-circles-formula
    // treat the bigger circle with p1 center
    // and smaller circle with p2 center
    // create new circle with r3 = r1 - r2 radius
    // inside the bigger p1 circle

    // distance between p1 and p2 circles
    double r3 = r2 - r1;
    double hyp = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    double tangent_length = std::sqrt(hyp * hyp - r3 * r3);
    double phi = std::atan2(y2 - y1, x2 - x1) + std::acos(r3 / hyp);

    // first outer tangent point on start car
    Point t_start = tangent_point(x1, y1, r1, phi);

    // first outer tangent point on goal car
    Point t_goal = tangent_point(x2, y2, r2, phi);

    // make first tangent line
    TangentLine outrt_line1(t_start, t_goal);

    // second outer tangent point on start car
    phi = std::atan2(y2 - y1, x2 - x1) - std::acos(r3 / hyp);

    t_start = tangent_point(x1, y1, r1, phi);

    // second outer tangent point on goal car
    t_goal = tangent_point(x2, y2, r2, phi);

    // make second tangent line
    TangentLine outer_line2(t_start, t_goal);

    return OuterTangentLines(outrt_line1, outer_line2);
}

double Simulation::arcLength(const double x1, const double y1,
                             const double x2, const double y2,
                             const double cx, const double cy,
                             const double r,
                             double& theta){
    // angle of vector aligned circle's center to car position
    Point v1(x1 - cx, y1 - cy);
    theta = std::atan2(v1.y, v1.x);

    // angle of vector aligned circel's center to tangent point
    cv::Point v2(x2 - cx, y2 - cy);
    double theta2 = std::atan2(v2.y, v2.x);

    // rotation angle
    double rotation = theta - theta2;
    return std::abs(rotation * r);
}

void Simulation::updateCar(DubinsCar& car, double delta, CarCommands command){
    double current_x = car.position.x;
    double current_y = car.position.y;
    double current_theta = -car.direction; // because OpenCV has Y-axis pointing down

    current_x = current_x + delta * std::cos(current_theta);
    current_y = current_y + delta * std::sin(current_theta);

    if(command == CarCommands::CAR_RIGHT){
        current_theta = current_theta + delta / car.radius;
    }

    car.position.x = current_x;
    car.position.y = current_y;
    car.direction = -current_theta;
}

void Simulation::run(
    cv::Mat& img
){
    // draw initial position and direction
    int direction_length = 100;
    cv::Point p;
    p.x = m_start.position.x + direction_length * std::cos(m_start.direction);
    p.y = m_start.position.y - direction_length * std::sin(m_start.direction);
    cv::line(img, cv::Point(m_start.position.x, m_start.position.y), 
        p, cv::Scalar(0, 0, 255), 1);

    // draw goal position and direction
    p.x = m_goal.position.x + direction_length * std::cos(m_goal.direction);
    p.y = m_goal.position.y - direction_length * std::sin(m_goal.direction);
    cv::line(img, cv::Point(m_goal.position.x, m_goal.position.y),
        p, cv::Scalar(0, 0, 255, 1));

    // draw initial point
    cv::circle(img, cv::Point(m_start.position.x, m_start.position.y), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);

    // draw circle to the right side of start position
    Point start_center;
    start_center.x = m_start.position.x + m_start.radius * std::cos(m_start.direction - M_PI_2);
    start_center.y = m_start.position.y - m_start.radius * std::sin(m_start.direction - M_PI_2);
    m_start.aligned_circle.center = start_center;
    cv::circle(img, cv::Point(start_center.x, start_center.y), m_start.radius, cv::Scalar(0, 0, 255), 1);

    // draw circle to the left side of goal position
    Point goal_center;
    goal_center.x = m_goal.position.x + m_goal.radius * std::cos(m_goal.direction - M_PI_2);
    goal_center.y = m_goal.position.y - m_goal.radius * std::sin(m_goal.direction - M_PI_2);
    m_goal.aligned_circle.center = goal_center;
    cv::circle(img, cv::Point(goal_center.x, goal_center.y), m_goal.radius, cv::Scalar(0, 0, 255), 1);

    // calculate tangent lines
    OuterTangentLines outer_lines = calculateTangentLines(m_start.aligned_circle.center.x, 
                                                          m_start.aligned_circle.center.y,
                                                          m_start.radius,
                                                          m_goal.aligned_circle.center.x,
                                                          m_goal.aligned_circle.center.y,
                                                          m_goal.radius);

    // draw tangent line
    TangentLine tline = outer_lines.second;
    Point tline_start = tline.first;
    Point tline_end = tline.second;
    cv::line(img, cv::Point(tline_start.x, tline_start.y), cv::Point(tline_end.x, tline_end.y), cv::Scalar(0, 0, 255), 1);

    // calculate arc length for start circle position
    double theta = 0.0;
    double arc1 = arcLength(m_start.position.x, m_start.position.y, tline_start.x, tline_start.y,
                            m_start.aligned_circle.center.x, m_start.aligned_circle.center.y,
                            m_start.radius, theta);
    m_start.arc_theta = theta;

    // calculate how many times to turn right        
    double delta = 0.01;
    int rotation_number = arc1 / delta;

    while(rotation_number--){
        updateCar(m_start, delta, CarCommands::CAR_RIGHT);
        cv::circle(img, cv::Point(m_start.position.x, m_start.position.y), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);
    }

    // move forward
    auto tp1 = tline.first;
    auto tp2 = tline.second;
    double line_dist = std::sqrt(std::pow(tp2.x - tp1.x, 2) + std::pow(tp2.y - tp1.y, 2));
    int straight_forward_number = line_dist / delta;

    while(straight_forward_number--){
        updateCar(m_start, delta, CarCommands::CAR_STRAIGHT);
        cv::circle(img, cv::Point(m_start.position.x, m_start.position.y), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);
    }

    double arc2 = arcLength(m_goal.position.x, m_goal.position.y, tline_end.x, tline_end.y,
                            m_goal.aligned_circle.center.x, m_goal.aligned_circle.center.y,
                            m_goal.radius, theta);
    m_goal.arc_theta = theta;

    rotation_number = arc2 / delta;
    while(rotation_number--){
        updateCar(m_start, delta, CarCommands::CAR_RIGHT);
        cv::circle(img, cv::Point(m_start.position.x, m_start.position.y), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);
    }
}