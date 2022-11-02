
#ifndef FOURBARMECHANISM_H
#define FOURBARMECHANISM_H

#include "Link.h"
#include "CouplerHead.h"

class FourBarMechanism
{
public:
    // Simply assembles the mechanism based on the 4 parts. Makes no checks
    FourBarMechanism(Link input_link, Link coupler_link, Link output_link, CouplerHead coupler_head);
    // Will generate the mechanism based on the positions of 3 coupler heads.
    // Input, coupler and output links will be generated automatically to fit the 3 positions.
    FourBarMechanism(CouplerHead coupler_head1, CouplerHead coupler_head2, CouplerHead coupler_head3, double linear_density);

    void rotate(double angle, double dt);
    double getTotalEnergy();
    std::string dumpState();
    std::string getDumpHeader();
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> getInputLinkPositions() const;
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> getCouplerLinkPositions() const;
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> getOutputLinkPositions() const;
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> getCouplerHeadTopPositions() const;

    static constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> getLinkPositionsFromCouplers(
        const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_1,
        const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_2,
        const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_3);

private:
    Link input_link;
    Link coupler_link;
    Link output_link;
    CouplerHead coupler_head;
    static constexpr double GRAVITY = 9.80665;
};
#endif