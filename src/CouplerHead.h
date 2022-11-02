#ifndef COUPLERHEAD_H
#define COUPLERHEAD_H
#include <tuple>
#include "Link.h"

class CouplerHead
{
public:
    CouplerHead();
    CouplerHead(Link &crank_link, Link &output_link, std::tuple<double, double> crank_top_point, std::tuple<double, double> output_top_point, double linear_density);

    void move(std::tuple<double, double> crank_link_pos, std::tuple<double, double> output_link_pos, double dt);
    std::tuple<double, double> getCrankPos();
    std::tuple<double, double> getOutputPos();
    std::tuple<double, double> getCrankTopPos() const;
    std::tuple<double, double> getOutputTopPos() const;
    double getEnergy();
    std::tuple<std::tuple<double, double>, std::tuple<double, double>> getBaseCouplerPositions();

private:
    void setEnergy(double speed, double angular_speed, double y_coord);
    double m;
    double im;
    double energy;
    double mass_linear_density;
    double l_c_ct;
    double m_c_ct;
    double l_ct_ot;
    double m_ct_ot;
    double l_ot_o;
    double m_ot_o;
    double l_c_o;
    double m_c_o;

    static constexpr double GRAVITY = 9.80665;
    std::tuple<double, double> crank_point;
    std::tuple<double, double> output_point;
    // Crank_top is the reference point
    std::tuple<double, double> crank_top_point;
    std::tuple<double, double> output_top_point;
};
#endif