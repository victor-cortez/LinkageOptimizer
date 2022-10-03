#include <cmath>
#include <tuple>
#include <utility>
#include <iostream>
#include "CouplerHead.h"

// Boiler plate code to make tuples add together
template <typename... T1, typename... T2, std::size_t... I>
constexpr auto add(const std::tuple<T1...> &t1, const std::tuple<T2...> &t2,
                   std::index_sequence<I...>)
{
    return std::tuple{std::get<I>(t1) + std::get<I>(t2)...};
}

template <typename... T1, typename... T2>
constexpr auto operator+(const std::tuple<T1...> &t1, const std::tuple<T2...> &t2)
{
    // make sure both tuples have the same size
    static_assert(sizeof...(T1) == sizeof...(T2));

    return add(t1, t2, std::make_index_sequence<sizeof...(T1)>{});
}

// End of boilerplate code

CouplerHead::CouplerHead() {}

// crank_link is the reference
CouplerHead::CouplerHead(Link &crank_link, Link &output_link, std::tuple<double, double> crank_top_point, std::tuple<double, double> input_top_point)
{
    this->crank_point = crank_link.getPos2();
    this->output_point = output_link.getPos();
    this->crank_top_point = crank_top_point + crank_link.getPos2();
    this->output_top_point = input_top_point + crank_link.getPos2();
    double linear_density = 1.0;
    double l1 = std::sqrt(std::pow(std::get<0>(this->crank_point) - std::get<0>(this->crank_top_point), 2) + std::pow(std::get<1>(this->crank_point) - std::get<1>(this->crank_top_point), 2));
    double l2 = std::sqrt(std::pow(std::get<0>(this->output_point) - std::get<0>(this->output_top_point), 2) + std::pow(std::get<1>(this->output_point) - std::get<1>(this->output_top_point), 2));
    double l3 = std::sqrt(std::pow(std::get<0>(this->crank_point) - std::get<0>(this->output_point), 2) + std::pow(std::get<1>(this->crank_point) - std::get<1>(this->output_point), 2));
    double l4 = std::sqrt(std::pow(std::get<0>(this->crank_top_point) - std::get<0>(this->output_top_point), 2) + std::pow(std::get<1>(this->crank_top_point) - std::get<1>(this->output_top_point), 2));
    this->m = (l1 + l2 + l3 + l4) * linear_density;
}

std::tuple<double, double> CouplerHead::move(std::tuple<double, double> crank_link_pos, std::tuple<double, double> output_link_pos, double dt)
{
    // Old position
    auto [xc, yc] = this->crank_point;
    auto [x2, y2] = this->output_point;

    // New position
    auto [xcn, ycn] = crank_link_pos;
    auto [xon, yon] = output_link_pos;

    double coupler_angle = atan2(y2 - yc, x2 - xc);
    double coupler_angle_new = atan2(yon - ycn, xon - xcn);

    // Link between crank and crank_top
    auto [xct, yct] = this->crank_top_point;
    double length_ct = std::sqrt(std::pow(xct - xc, 2) + std::pow(yct - yc, 2));
    // Angle between the two points relative to the coupler
    double angle_ct = std::atan2(yct - yc, xct - xc) - coupler_angle;
    // std::cout << "angle_ct: " << angle_ct << std::endl;
    double xctn = xcn + length_ct * std::cos(angle_ct + coupler_angle_new);
    double yctn = ycn + length_ct * std::sin(angle_ct + coupler_angle_new);

    // Link between output and output_top
    auto [xot, yot] = this->output_top_point;
    double length_ot = std::sqrt(std::pow(xot - x2, 2) + std::pow(yot - y2, 2));
    // Angle between the two points relative to the coupler
    double angle_ot = std::atan2(yot - y2, xot - x2) - coupler_angle;
    // std::cout << "angle_ot: " << angle_ot << "\n";
    double xotn = xon + length_ot * std::cos(angle_ot + coupler_angle_new);
    double yotn = yon + length_ot * std::sin(angle_ot + coupler_angle_new);

    // Link between crank_top and input_top is predefined by the constant angles condition

    // TODO: add calculations for speed and angular speed

    this->crank_point = crank_link_pos;
    this->output_point = output_link_pos;
    this->crank_top_point = std::make_tuple(xctn, yctn);
    this->output_top_point = std::make_tuple(xotn, yotn);
}

std::tuple<double, double> CouplerHead::getCrankPos()
{
    return this->crank_point;
}

std::tuple<double, double> CouplerHead::getOutputPos()
{
    return this->output_point;
}

std::tuple<double, double> CouplerHead::getCrankTopPos()
{
    return this->crank_top_point;
}

std::tuple<double, double> CouplerHead::getOutputTopPos()
{
    return this->output_top_point;
}

std::tuple<std::tuple<double, double>, std::tuple<double, double>> CouplerHead::getBaseCouplerPositions()
{
    return std::make_tuple(this->crank_point, this->output_point);
}