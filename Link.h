#ifndef LINK_H
#define LINK_H
#include <tuple>

class Link
{
public:
    Link(Link &link);
    // Regular Link
    Link(Link &past_link, double mass, double im, double length);
    // Regular Link
    Link(std::tuple<double, double> pos, std::tuple<double, double> pos2, double mass, double im);
    // Regular Link
    Link(double mass, double im, double length);
    // Ground Link

    void setPastLink(Link &past_link);
    Link &getPastLink();
    void setNextLink(Link &next_link);
    Link &getNextLink();

    void setIM(double im_value);
    double getIM();
    void setM(double m_value);
    double getM();
    void setL(double l_value);
    double getL();

    std::tuple<double, double> setTheta(double theta_value, double dt);
    double getTheta();
    // Pos always refer to the closest connection to the crank link (on the left usually)
    std::tuple<double, double> setPos(std::tuple<double, double> pos_value, double dt);
    std::tuple<double, double> getPos();

    // Other joint (the right side)
    std::tuple<double, double> setPos2(std::tuple<double, double> pos_value, double dt);
    std::tuple<double, double> getPos2();

    // Set both positions
    std::tuple<double, double> setTwoPositions(std::tuple<double, double> pos_value, std::tuple<double, double> pos_value2, double dt);

private:
    Link *past_link;
    Link *next_link;

    std::tuple<double, double> position;
    std::tuple<double, double> position2;
    double m;
    double im;
    double length;
    double angle;
    bool is_ground;
};
#endif