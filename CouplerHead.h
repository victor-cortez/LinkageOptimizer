#ifndef COUPLERHEAD_H
#define COUPLERHEAD_H
#include <tuple>
#include "Link.h"

class CouplerHead
{
public:
    CouplerHead(Link &crank_link, Link &output_link, std::tuple<double, double> crank_top_point, std::tuple<double, double> output_top_point);

    std::tuple<double, double> move(std::tuple<double, double> crank_link_pos, std::tuple<double, double> output_link_pos, double dt);
    std::tuple<double, double> getCrankPos();
    std::tuple<double, double> getOutputPos();
    std::tuple<double, double> getCrankTopPos();
    std::tuple<double, double> getOutputTopPos();

private:
    double m;
    std::tuple<double, double> crank_point;
    std::tuple<double, double> output_point;
    // Crank_top is the reference point
    std::tuple<double, double> crank_top_point;
    std::tuple<double, double> output_top_point;
};
#endif