#include <cmath>
#include <tuple>
#include "Link.h"

Link::Link() {}

Link::Link(Link &link)
{
    this->past_link = link.past_link;
    this->next_link = link.next_link;
    this->position = link.position;
    this->position2 = link.position2;
    this->m = link.m;
    this->im = link.im;
    this->length = link.length;
    this->angle = link.angle;
    this->is_ground = link.is_ground;
}
Link::Link(std::tuple<double, double> pos, std::tuple<double, double> pos2, double mass, double im)
{
    position = pos;
    position2 = pos2;
    auto [x, y] = pos;
    auto [x2, y2] = pos2;
    angle = atan2(y2 - y, x2 - x);
    length = sqrt(pow(x2 - x, 2) + pow(y2 - y, 2));
    m = mass;
    im = im;
    is_ground = false;
}

std::tuple<double, double> Link::setTheta(double theta_value, double dt)
{
    auto [x, y] = position;
    auto [x2, y2] = position2;
    double cm_x = (x + x2) / 2.0;
    double cm_y = (y + y2) / 2.0;

    double x2n = x + this->length * cos(theta_value);
    double y2n = y + this->length * sin(theta_value);

    double cm_xn = (x + x2n) / 2.0;
    double cm_yn = (y + y2n) / 2.0;

    double speed = (std::sqrt(std::pow(cm_xn - cm_x, 2) + std::pow(cm_yn - cm_y, 2))) / dt;
    double angular_speed = (theta_value - this->angle) / dt;

    this->angle = theta_value;
    this->position2 = std::make_tuple(x2n, y2n);
    return std::make_tuple(speed, angular_speed);
}
double Link::getTheta()
{
    return this->angle;
}

std::tuple<double, double> Link::setPos(std::tuple<double, double> pos_value, double dt)
{
    auto [x, y] = this->position;
    auto [x2, y2] = this->position2;
    auto [xn, yn] = pos_value;
    double length_new = std::sqrt(std::pow(xn - x2, 2) + std::pow(yn - y2, 2));
    double angle_new = std::atan2(y2 - yn, x2 - xn);

    double cm_x = (x + x2) / 2.0;
    double cm_y = (y + y2) / 2.0;
    double cm_xn = (xn + x2) / 2.0;
    double cm_yn = (yn + y2) / 2.0;
    double speed = (std::sqrt(std::pow(cm_xn - cm_x, 2) + std::pow(cm_yn - cm_y, 2))) / dt;
    double angular_speed = (angle_new - this->angle) / dt;

    this->position = pos_value;
    this->angle = angle_new;
    this->length = length_new;
    return std::make_tuple(speed, angular_speed);
}

std::tuple<double, double> Link::getPos()
{
    return this->position;
}

std::tuple<double, double> Link::getPos2()
{
    return this->position2;
}
std::tuple<double, double> Link::setTwoPositions(std::tuple<double, double> pos_value, std::tuple<double, double> pos_value2, double dt)
{
    auto [x, y] = this->position;
    auto [x2, y2] = this->position2;
    auto [xn, yn] = pos_value;
    auto [xn2, yn2] = pos_value2;

    double length_new = std::sqrt(std::pow(xn - xn2, 2) + std::pow(yn - yn2, 2));
    double angle_new = std::atan2(yn2 - yn, xn2 - xn);

    double cm_x = (x + x2) / 2.0;
    double cm_y = (y + y2) / 2.0;
    double cm_xn = (xn + xn2) / 2.0;
    double cm_yn = (yn + yn2) / 2.0;

    double speed = (std::sqrt(std::pow(cm_xn - cm_x, 2) + std::pow(cm_yn - cm_y, 2))) / dt;
    double angular_speed = (angle_new - this->angle) / dt;

    this->position = pos_value;
    this->position2 = pos_value2;
    this->angle = angle_new;
    this->length = length_new;

    return std::make_tuple(speed, angular_speed);
}

double Link::getL()
{
    return this->length;
}

double Link::getM()
{
    return this->m;
}

double Link::getIM()
{
    return this->im;
}