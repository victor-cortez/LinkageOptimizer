
#include <cmath>
#include <optional>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include "FourBarMechanism.h"

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
    std::string message;

public:
    CirclesDoNotIntersect(std::string msg) : message(msg) {}
    char *what()
    {
        return message.data();
    }
};

// TODO
// Recode this for copyright reasons
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

double FourBarMechanism::getTotalEnergy()
{
    return input_link.getEnergy() + output_link.getEnergy() + coupler_link.getEnergy() + coupler_head.getEnergy();
}

FourBarMechanism::FourBarMechanism(Link crank_link, Link in_coupler_link, Link in_output_link, CouplerHead in_coupler_head) : input_link(crank_link),
                                                                                                                              coupler_link(in_coupler_link),
                                                                                                                              output_link(in_output_link),
                                                                                                                              coupler_head(in_coupler_head)
{
}

constexpr double det(const double matrix[3][3])
{
    return matrix[0][0] * matrix[1][1] * matrix[2][2] + matrix[0][1] * matrix[1][2] * matrix[2][0] + matrix[0][2] * matrix[1][0] * matrix[2][1] - matrix[0][2] * matrix[1][1] * matrix[2][0] - matrix[0][1] * matrix[1][0] * matrix[2][2] - matrix[0][0] * matrix[1][2] * matrix[2][1];
}

class CouplerRootPointsAreColinear : public std::exception
{
private:
    std::string message;

public:
    CouplerRootPointsAreColinear(std::string msg) : message(msg) {}
    char *what()
    {
        return message.data();
    }
};

constexpr std::tuple<double, double> getCircleCenter(const std::tuple<double, double> a1, const std::tuple<double, double> a2, const std::tuple<double, double> a3)
{
    // We know that the distance from our point x,y must be equal to r in each of the 3 points
    // The formula of the circle is
    // x^2 + y^2 + ax + by + c = 0
    // But the same formula based on A,B root points and radius is
    // (x-A)^2 + (y-B)^2 = R^2
    // x^2 - 2Ax + A^2 + y^2 - 2By + B^2 = R^2
    // x^2 + y^2 + (-2A)x + (-2BY) + (A^2  + B^2  - R^2) = 0
    // With a = -2A, b = -2B, c = A^2 + B^2 - R^2
    // Our Equations are therefore
    // a*a1 + b*b1 + c = -(a1^2+b1^2)
    // a*a2 + b*b2 + c = -(a2^2+b2^2)
    // a*a3 + b*b3 + c = -(a3^2+b3^2)

    // Given our 3 results d1,d2,d3
    double d1 = -(std::pow(std::get<0>(a1), 2) + std::pow(std::get<1>(a1), 2));
    double d2 = -(std::pow(std::get<0>(a2), 2) + std::pow(std::get<1>(a2), 2));
    double d3 = -(std::pow(std::get<0>(a3), 2) + std::pow(std::get<1>(a3), 2));

    // Our Cramer's rule matrices are
    double D[3][3] = {
        {std::get<0>(a1), std::get<1>(a1), 1},
        {std::get<0>(a2), std::get<1>(a2), 1},
        {std::get<0>(a3), std::get<1>(a3), 1}};
    double Dx[3][3] = {
        {d1, std::get<1>(a1), 1},
        {d2, std::get<1>(a2), 1},
        {d3, std::get<1>(a3), 1}};
    double Dy[3][3] = {
        {std::get<0>(a1), d1, 1},
        {std::get<0>(a2), d2, 1},
        {std::get<0>(a3), d3, 1}};
    // double Dz[3][3] = {
    // {std::get<0>(a1), std::get<1>(a1), d1},
    // {std::get<0>(a2), std::get<1>(a2), d2},
    // {std::get<0>(a3), std::get<1>(a3), d3}};

    // Now we calculate our coefficients
    double detD = det(D);
    if (detD == 0)
    {
        throw CouplerRootPointsAreColinear("The coupler root points are colinear. Division by Zero error iminent!");
    }
    double detDx = det(Dx);
    double detDy = det(Dy);
    //  double detDz = det(Dz);

    double a = detDx / detD;
    double b = detDy / detD;
    // double c = detDz / detD;

    double x = -a / 2;
    double y = -b / 2;
    return std::make_tuple(x, y);
}

constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> FourBarMechanism::getLinkPositionsFromCouplers(
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_1,
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_2,
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_3)
{
    const std::tuple<double, double> a1 = std::get<0>(coupler_pos_1);
    const std::tuple<double, double> b1 = std::get<1>(coupler_pos_1);
    const std::tuple<double, double> a2 = std::get<0>(coupler_pos_2);
    const std::tuple<double, double> b2 = std::get<1>(coupler_pos_2);
    const std::tuple<double, double> a3 = std::get<0>(coupler_pos_3);
    const std::tuple<double, double> b3 = std::get<1>(coupler_pos_3);

    const std::tuple<double, double> crank_link_root = getCircleCenter(a1, a2, a3);
    const std::tuple<double, double> output_link_root = getCircleCenter(b1, b2, b3);

    return std::make_tuple(crank_link_root, output_link_root);
}

FourBarMechanism::FourBarMechanism(CouplerHead coupler_head1, CouplerHead coupler_head2, CouplerHead coupler_head3, double linear_density)
{
    std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_1 = coupler_head1.getBaseCouplerPositions();
    std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_2 = coupler_head2.getBaseCouplerPositions();
    std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_3 = coupler_head3.getBaseCouplerPositions();
    auto [crank_link_root, output_link_root] = getLinkPositionsFromCouplers(coupler_pos_1, coupler_pos_2, coupler_pos_3);
    input_link = Link(crank_link_root, std::get<0>(coupler_pos_1), linear_density);
    output_link = Link(std::get<1>(coupler_pos_1), output_link_root, linear_density);
    coupler_link = Link(std::get<0>(coupler_pos_1), std::get<1>(coupler_pos_1), linear_density);
    coupler_head = coupler_head1;
}

FourBarMechanism::FourBarMechanism(const FourBarMechanism &other)
{
    this->input_link = other.input_link;
    this->output_link = other.output_link;
    this->coupler_link = other.coupler_link;
    this->coupler_head = other.coupler_head;
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
    return std::string("theta,xi,yi,xc,yc,xo,yo,xb,yb,xct,yct,xot,yot,energy");
}

std::string FourBarMechanism::dumpState()
{
    // Create an output string stream
    std::ostringstream streamObj;
    // Set Fixed -Point Notation
    streamObj << std::fixed;
    // Set precision to 2 digits
    streamObj << std::setprecision(6);
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
    streamObj << std::get<1>(this->coupler_head.getOutputTopPos()) << ",";
    streamObj << getTotalEnergy();
    // Get string from output string stream
    return streamObj.str();
}

double FourBarMechanism::getAngle()
{
    return this->input_link.getTheta();
}

const std::tuple<std::tuple<double, double>, std::tuple<double, double>> FourBarMechanism::getInputLinkPositions() const
{
    return std::make_tuple(this->input_link.getPos(), this->input_link.getPos2());
}

const std::tuple<std::tuple<double, double>, std::tuple<double, double>> FourBarMechanism::getCouplerLinkPositions() const
{
    return std::make_tuple(this->coupler_link.getPos(), this->coupler_link.getPos2());
}

const std::tuple<std::tuple<double, double>, std::tuple<double, double>> FourBarMechanism::getOutputLinkPositions() const
{
    return std::make_tuple(this->output_link.getPos(), this->output_link.getPos2());
}

const std::tuple<std::tuple<double, double>, std::tuple<double, double>> FourBarMechanism::getCouplerHeadTopPositions() const
{
    return std::make_tuple(this->coupler_head.getCrankTopPos(), this->coupler_head.getOutputTopPos());
}