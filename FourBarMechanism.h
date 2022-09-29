
#ifndef FOURBARMECHANISM_H
#define FOURBARMECHANISM_H

#include "Link.h"
#include "CouplerHead.h"

class FourBarMechanism
{
public:
    FourBarMechanism(std::tuple<double, double> button_pair1, std::tuple<double, double> button_pair2, std::tuple<double, double> button_pair3);
    FourBarMechanism(Link input_link, Link coupler_link, Link output_link, CouplerHead coupler_head);
    void rotate(double angle, double dt);
    std::string dumpState();
    std::string getDumpHeader();

private:
    Link &input_link;
    Link &output_link;
    Link &coupler_link;
    CouplerHead &coupler_head;
};
#endif