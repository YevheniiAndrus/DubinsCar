#include "simulation.hpp"

Simulation::Simulation(
    const DubinsCar& start,
    const DubinsCar& goal) :
    m_start(start), 
    m_goal(goal) {}

OuterTangentLines Simulation::calculateTangentLines(
    const DubinsCar& start,
    const DubinsCar& goal
){
    // lambda to calculate tangent point given Dubins car and angle
    // between vector start to goal position and tangent point
    auto tangent_point = [](const DubinsCar& car, double phi){
        cv::Point tangent;
        tangent.x = car.aligned_circle.center.x + car.radius * std::cos(phi);
        tangent.y = car.aligned_circle.center.y + car.radius * std::sin(phi);
        return tangent;
    };

    // math taken from here - https://math.stackexchange.com/questions/719758/inner-tangent-between-two-circles-formula
    // treat the bigger circle with p1 center
    // and smaller circle with p2 center
    // create new circle with r3 = r1 - r2 radius
    // inside the bigger p1 circle

    // distance between p1 and p2 circles
    double r3 = start.radius - goal.radius;
    double hyp = std::sqrt(std::pow(start.aligned_circle.center.x - goal.aligned_circle.center.x, 2) +
                      std::pow(start.aligned_circle.center.y - goal.aligned_circle.center.y, 2));
    double tangent_length = std::sqrt(hyp * hyp - r3 * r3);

    double phi = std::atan2(goal.position.y - start.position.y, goal.position.x - start.position.x) 
                 + std::acos(r3 / hyp);

    // first outer tangent point on start car
    cv::Point t_start = tangent_point(start, phi);

    // first outer tangent point on goal car
    cv::Point t_goal = tangent_point(goal, phi);

    // make first tangent line
    TangentLine outrt_line1(t_start, t_goal);

    // second outer tangent point on start car
    phi = std::atan2(goal.position.y - start.position.y, goal.position.x - start.position.x) 
                 - std::acos(r3 / hyp);

    t_start = tangent_point(start, phi);

    // second outer tangent point on goal car
    t_goal = tangent_point(goal, phi);

    // make second tangent line
    TangentLine outer_line2(t_start, t_goal);

    return OuterTangentLines(outrt_line1, outer_line2);
}

double Simulation::arcLength(DubinsCar& car, const cv::Point& tangent_point){
    // angle of vector aligned circle's center to car position
    cv::Point v1(car.aligned_circle.center.x - car.position.x,
                 car.aligned_circle.center.y - car.position.y);
    double theta1 = std::atan2(v1.y, v1.x);
    car.arc_theta = theta1;
    std::cout << "theta1: " << theta1 << std::endl;

    // angle of vector aligned circel's center to tangent point
    cv::Point v2(car.aligned_circle.center.x - tangent_point.x,
                 car.aligned_circle.center.y - tangent_point.y);
    double theta2 = std::atan2(v2.y, v2.x);
    std::cout << "theta2: " << theta2 << std::endl;

    // rotation angle
    double rotation = theta1 - theta2;
    std::cout << "rotation: " << rotation << std::endl;
    return std::abs(rotation * car.radius);
}

void Simulation::updateCar(DubinsCar& car, double delta, CarCommands command){
    double current_x = car.position.x;
    double current_y = car.position.y;
    double current_theta = car.arc_theta;

    current_x = current_x + delta * std::cos(current_theta);
    current_y = current_y + delta * std::sin(current_theta);

    if(command == CarCommands::CAR_RIGHT){
        current_theta = current_theta + delta / car.radius;
    }

    car.position.x = current_x;
    car.position.y = current_y;
    car.arc_theta = current_theta;
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
    cv::Point start_center;
    start_center.x = m_start.position.x + m_start.radius * std::cos(m_start.direction - M_PI_2);
    start_center.y = m_start.position.y - m_start.radius * std::sin(m_start.direction - M_PI_2);
    m_start.aligned_circle.center = start_center;
    cv::circle(img, start_center, m_start.radius, cv::Scalar(0, 0, 255), 1);

    // draw circle to the left side of goal position
    cv::Point goal_center;
    goal_center.x = m_goal.position.x + m_goal.radius * std::cos(m_goal.direction - M_PI_2);
    goal_center.y = m_goal.position.y + m_goal.radius * std::sin(m_goal.direction - M_PI_2);
    m_goal.aligned_circle.center = goal_center;
    cv::circle(img, goal_center, m_goal.radius, cv::Scalar(0, 0, 255), 1);

    // calculate tangent lines
    OuterTangentLines outer_lines = calculateTangentLines(m_start, m_goal);

    // draw tangent line
    TangentLine tline = outer_lines.second;
    cv::Point tline_start = tline.first;
    cv::Point tline_end = tline.second;
    cv::line(img, tline_start, tline_end, cv::Scalar(0, 0, 255), 1);

    // calculate arc length for start circle position
    double arc1 = arcLength(m_start, tline_start);
    std::cout << "Arc1 len: " << arc1 << std::endl;

    // // calculate how many times to turn right
    double delta = 0.01;
    int rotation_number = static_cast<int>(arc1 / 0.01);
    std::cout << "rotation number: " << rotation_number << std::endl;

    while(rotation_number){
        updateCar(m_start, delta, CarCommands::CAR_RIGHT);
        cv::circle(img, cv::Point(m_start.position.x, m_start.position.y), 2, cv::Scalar(0, 255, 0), 2, cv::LineTypes::FILLED);
        rotation_number--;
    }
}