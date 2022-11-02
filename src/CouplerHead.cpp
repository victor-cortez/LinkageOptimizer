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
CouplerHead::CouplerHead(Link &crank_link, Link &output_link, std::tuple<double, double> crank_top_point, std::tuple<double, double> input_top_point, double linear_density)
{
    this->crank_point = crank_link.getPos2();
    this->output_point = output_link.getPos();
    this->crank_top_point = crank_top_point + crank_link.getPos2();
    this->output_top_point = input_top_point + crank_link.getPos2();

    auto [xc, yc] = this->crank_point;
    auto [xo, yo] = this->output_point;
    auto [xct, yct] = this->crank_top_point;
    auto [xot, yot] = this->output_top_point;

    double l_c_ct = std::sqrt(std::pow(std::get<0>(this->crank_point) - std::get<0>(this->crank_top_point), 2) + std::pow(std::get<1>(this->crank_point) - std::get<1>(this->crank_top_point), 2));
    double l_o_ot = std::sqrt(std::pow(std::get<0>(this->output_point) - std::get<0>(this->output_top_point), 2) + std::pow(std::get<1>(this->output_point) - std::get<1>(this->output_top_point), 2));
    double l_c_o = std::sqrt(std::pow(std::get<0>(this->crank_point) - std::get<0>(this->output_point), 2) + std::pow(std::get<1>(this->crank_point) - std::get<1>(this->output_point), 2));
    double l_ct_ot = std::sqrt(std::pow(std::get<0>(this->crank_top_point) - std::get<0>(this->output_top_point), 2) + std::pow(std::get<1>(this->crank_top_point) - std::get<1>(this->output_top_point), 2));

    double m_c_ct = l_c_ct * linear_density;
    double m_o_ot = l_o_ot * linear_density;
    double m_c_o = l_c_o * linear_density;
    double m_ct_ot = l_ct_ot * linear_density;

    this->m = m_c_ct + m_o_ot + m_c_o + m_ct_ot;

    double centroid_c_ct_x = (xc + xct) / 2.0;
    double centroid_c_ct_y = (yc + yct) / 2.0;
    double centroid_ct_ot_x = (xct + xot) / 2.0;
    double centroid_ct_ot_y = (yct + yot) / 2.0;
    double centroid_ot_o_x = (xot + xo) / 2.0;
    double centroid_ot_o_y = (yot + yo) / 2.0;
    double centroid_c_o_x = (xc + xo) / 2.0;
    double centroid_c_o_y = (yc + yo) / 2.0;

    double centroid_x = (centroid_c_ct_x * m_c_ct / 2.0) + (centroid_ct_ot_x * m_ct_ot / 2.0) + (centroid_ot_o_x * m_ot_o / 2.0) + (centroid_c_o_x * m_c_o / 2.0);
    double centroid_y = (centroid_c_ct_y * m_c_ct / 2.0) + (centroid_ct_ot_y * m_ct_ot / 2.0) + (centroid_ot_o_y * m_ot_o / 2.0) + (centroid_c_o_y * m_c_o / 2.0);

    double dist_c_ct = std::sqrt(std::pow(centroid_c_ct_x - centroid_x, 2) + std::pow(centroid_c_ct_y - centroid_y, 2));
    double dist_ct_ot = std::sqrt(std::pow(centroid_ct_ot_x - centroid_x, 2) + std::pow(centroid_ct_ot_y - centroid_y, 2));
    double dist_ot_o = std::sqrt(std::pow(centroid_ot_o_x - centroid_x, 2) + std::pow(centroid_ot_o_y - centroid_y, 2));
    double dist_c_o = std::sqrt(std::pow(centroid_c_o_x - centroid_x, 2) + std::pow(centroid_c_o_y - centroid_y, 2));

    // Calculating the moments of intertia in relation to the centroid of the entire coupler head
    // Using the parallel axis theorem
    double im_c_ct = m_c_ct * ((std::pow(l_c_ct, 2) / 12.0) + std::pow(dist_c_ct, 2));
    double im_ct_ot = m_ct_ot * ((std::pow(l_ct_ot, 2) / 12.0) + std::pow(dist_ct_ot, 2));
    double im_ot_o = m_ot_o * ((std::pow(l_ot_o, 2) / 12.0) + std::pow(dist_ot_o, 2));
    double im_c_o = m_c_o * ((std::pow(l_c_o, 2) / 12.0) + std::pow(dist_c_o, 2));

    this->im = im_c_ct + im_ct_ot + im_ot_o + im_c_o;
}

void CouplerHead::move(std::tuple<double, double> crank_link_pos, std::tuple<double, double> output_link_pos, double dt)
{
    // Old position
    auto [xc, yc] = this->crank_point;
    auto [xo, yo] = this->output_point;
    auto [xct, yct] = this->crank_top_point;
    auto [xot, yot] = this->output_top_point;
    double old_centroid_x = ((xc + xct) * m_c_ct / 2.0) + ((xct + xot) * m_ct_ot / 2.0) + ((xot + xo) * m_ot_o / 2.0) + ((xc + xo) * m_c_o / 2.0);
    double old_centroid_y = ((yc + yct) * m_c_ct / 2.0) + ((yct + yot) * m_ct_ot / 2.0) + ((yot + yo) * m_ot_o / 2.0) + ((yc + yo) * m_c_o / 2.0);

    // New position
    auto [xcn, ycn] = crank_link_pos;
    auto [xon, yon] = output_link_pos;

    double coupler_angle = atan2(yo - yc, xo - xc);
    double coupler_angle_new = atan2(yon - ycn, xon - xcn);

    // Link between crank and crank_top

    double length_ct = std::sqrt(std::pow(xct - xc, 2) + std::pow(yct - yc, 2));
    // Angle between the two points relative to the coupler
    double angle_ct = std::atan2(yct - yc, xct - xc) - coupler_angle;
    // std::cout << "angle_ct: " << angle_ct << std::endl;
    double xctn = xcn + length_ct * std::cos(angle_ct + coupler_angle_new);
    double yctn = ycn + length_ct * std::sin(angle_ct + coupler_angle_new);

    // Link between output and output_top

    double length_ot = std::sqrt(std::pow(xot - xo, 2) + std::pow(yot - yo, 2));
    // Angle between the two points relative to the coupler
    double angle_ot = std::atan2(yot - yo, xot - xo) - coupler_angle;
    // std::cout << "angle_ot: " << angle_ot << "\n";
    double xotn = xon + length_ot * std::cos(angle_ot + coupler_angle_new);
    double yotn = yon + length_ot * std::sin(angle_ot + coupler_angle_new);

    // Link between crank_top and input_top is predefined by the constant angles condition

    // TODO: add calculations for speed and angular speed
    double new_centroid_x = ((xcn + xctn) * m_c_ct / 2.0) + ((xctn + xotn) * m_ct_ot / 2.0) + ((xotn + xon) * m_ot_o / 2.0) + ((xcn + xon) * m_c_o / 2.0);
    double new_centroid_y = ((ycn + yctn) * m_c_ct / 2.0) + ((yctn + yotn) * m_ct_ot / 2.0) + ((yotn + yon) * m_ot_o / 2.0) + ((ycn + yon) * m_c_o / 2.0);
    double speed = std::sqrt(std::pow(new_centroid_x - old_centroid_x, 2) + std::pow(new_centroid_y - old_centroid_y, 2)) / dt;
    double angular_speed = (coupler_angle_new - coupler_angle) / dt;
    setEnergy(speed, angular_speed, new_centroid_y);

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

std::tuple<double, double> CouplerHead::getCrankTopPos() const
{
    return this->crank_top_point;
}

std::tuple<double, double> CouplerHead::getOutputTopPos() const
{
    return this->output_top_point;
}

std::tuple<std::tuple<double, double>, std::tuple<double, double>> CouplerHead::getBaseCouplerPositions()
{
    return std::make_tuple(this->crank_point, this->output_point);
}

void CouplerHead::setEnergy(double speed, double angular_speed, double y_coord)
{
    energy = (m * std::pow(speed, 2) / 2.0) + (im * std::pow(angular_speed, 2) / 2.0) + (m * GRAVITY * y_coord);
}

double CouplerHead::getEnergy()
{
    return this->energy;
}
