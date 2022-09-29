
#include <cmath>
#include <optional>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include "Link.h"
#include "FourBarMechanism.h"
#include "CouplerHead.h"

// To print tuples
template <class Ch, class Tr, class... Args>
auto &operator<<(std::basic_ostream<Ch, Tr> &os, std::tuple<Args...> const &t)
{
    std::basic_stringstream<Ch, Tr> ss;
    ss << "[ ";
    std::apply([&ss](auto &&...args)
               { ((ss << args << ", "), ...); },
               t);
    ss.seekp(-2, ss.cur);
    ss << " ]";
    return os << ss.str();
}

class CirclesDoNotIntersect : public std::exception
{
private:
    char *message;

public:
    CirclesDoNotIntersect(char *msg) : message(msg) {}
    char *what()
    {
        return message;
    }
};

std::optional<std::tuple<std::tuple<double, double>, std::tuple<double, double>>> intersectTwoCircles(double x1, double y1, double r1, double x2, double y2, double r2)
{
    double centerdx = x1 - x2;
    double centerdy = y1 - y2;
    double R = std::sqrt(centerdx * centerdx + centerdy * centerdy);
    if (!(std::abs(r1 - r2) <= R && R <= r1 + r2))
    {                                                                                                   // no intersection
        throw CirclesDoNotIntersect("The circles do not intersect, hence a linkage can not move here"); // empty list of results
    }
    // intersection(s) should exist

    double R2 = R * R;
    double R4 = R2 * R2;
    double a = (r1 * r1 - r2 * r2) / (2 * R2);
    double r2r2 = (r1 * r1 - r2 * r2);
    double c = std::sqrt(2 * (r1 * r1 + r2 * r2) / R2 - (r2r2 * r2r2) / R4 - 1);

    double fx = (x1 + x2) / 2 + a * (x2 - x1);
    double gx = c * (y2 - y1) / 2;
    double ix1 = fx + gx;
    double ix2 = fx - gx;

    double fy = (y1 + y2) / 2 + a * (y2 - y1);
    double gy = c * (x1 - x2) / 2;
    double iy1 = fy + gy;
    double iy2 = fy - gy;

    // note if gy == 0 and gx == 0 then the circles are tangent and there is only one solution
    // but that one solution will just be duplicated as the code is currently written
    return std::make_tuple(std::make_tuple(ix1, iy1), std::make_tuple(ix2, iy2));
}

double calculateEnergySpent(Link &link, double speed, double angular_speed)
{
    return (link.getM() * std::pow(speed, 2) / 2) + (link.getIM() * std::pow(angular_speed, 2) / 2);
}

FourBarMechanism::FourBarMechanism(Link crank_link, Link in_coupler_link, Link in_output_link, CouplerHead in_coupler_head) : input_link(crank_link),
                                                                                                                              coupler_link(in_coupler_link),
                                                                                                                              output_link(in_output_link),
                                                                                                                              coupler_head(in_coupler_head)
{
}

void FourBarMechanism::rotate(double angle, double dt)
{
    if (std::abs(angle - this->input_link.getTheta()) < 0.00001)
    {
        std::cout << " no move";
        return;
    }
    auto past_pin_joint_pos = this->output_link.getPos();
    this->input_link.setTheta(angle, dt);

    // The right end of the crank bar
    double x_crank = this->input_link.getL() * cos(this->input_link.getTheta());
    double y_crank = this->input_link.getL() * sin(this->input_link.getTheta());
    double coupler_lenght = this->coupler_link.getL();
    double output_link_length = this->output_link.getL();
    auto [x_ground_2, y_ground_2] = this->output_link.getPos2();
    auto possible_pin_joint_locations = intersectTwoCircles(x_crank, y_crank, coupler_lenght, x_ground_2, y_ground_2, output_link_length);

    // std::cout << past_pin_joint_pos << "\n";
    // std::cout << std::get<0>(possible_pin_joint_locations.value()) << " " << std::get<1>(possible_pin_joint_locations.value()) << "\n";

    auto [x_pin_joint_1, y_pin_joint_1] = std::get<0>(possible_pin_joint_locations.value());
    auto [x_pin_joint_2, y_pin_joint_2] = std::get<1>(possible_pin_joint_locations.value());
    auto [past_x_pin_joint, past_y_pin_joint] = past_pin_joint_pos;
    double distance_1 = std::sqrt(std::pow(x_pin_joint_1 - past_x_pin_joint, 2) + std::pow(y_pin_joint_1 - past_y_pin_joint, 2));
    double distance_2 = std::sqrt(std::pow(x_pin_joint_2 - past_x_pin_joint, 2) + std::pow(y_pin_joint_2 - past_y_pin_joint, 2));
    // The root of the outputlink is the crank. The end of the outpulink should still be fixed in the ground
    // std::cout << "distance 1: " << distance_1 << " distance 2: " << distance_2 << "\n";
    if (distance_1 < distance_2)
    {
        this->output_link.setPos(std::make_tuple(x_pin_joint_1, y_pin_joint_1), dt);
    }
    else
    {
        this->output_link.setPos(std::make_tuple(x_pin_joint_2, y_pin_joint_2), dt);
    }
    // The root of the coupler link is the tail of the crank link
    this->coupler_link.setTwoPositions(this->input_link.getPos2(), this->output_link.getPos(), dt);
    this->coupler_head.move(this->input_link.getPos2(), this->coupler_link.getPos2(), dt);
}

std::string FourBarMechanism::getDumpHeader()
{
    return std::string("theta,xi,yi,xc,yc,xo,yo,xb,yb,xct,yct,xot,yot\n");
}

std::string FourBarMechanism::dumpState()
{
    // Create an output string stream
    std::ostringstream streamObj;
    // Set Fixed -Point Notation
    streamObj << std::fixed;
    // Set precision to 2 digits
    streamObj << std::setprecision(4);
    // Add doubles to stream
    streamObj << this->input_link.getTheta() << ",";
    streamObj << std::get<0>(this->input_link.getPos()) << ",";
    streamObj << std::get<1>(this->input_link.getPos()) << ",";
    streamObj << std::get<0>(this->coupler_link.getPos()) << ",";
    streamObj << std::get<1>(this->coupler_link.getPos()) << ",";
    streamObj << std::get<0>(this->output_link.getPos()) << ",";
    streamObj << std::get<1>(this->output_link.getPos()) << ",";
    streamObj << std::get<0>(this->output_link.getPos2()) << ",";
    streamObj << std::get<1>(this->output_link.getPos2()) << ",";
    streamObj << std::get<0>(this->coupler_head.getCrankTopPos()) << ",";
    streamObj << std::get<1>(this->coupler_head.getCrankTopPos()) << ",";
    streamObj << std::get<0>(this->coupler_head.getOutputTopPos()) << ",";
    streamObj << std::get<1>(this->coupler_head.getOutputTopPos()) << "\n";
    // Get string from output string stream
    return streamObj.str();
}