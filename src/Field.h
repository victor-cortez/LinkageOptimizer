#ifndef FIELD_H
#define FIELD_H
#include <vector>
#include <tuple>
#include <initializer_list>

// USE A R THAT IS R_BUTTON/SQRT(2) OF THE BUTTON RADIUS R_BUTTON
// THIS IS TO MAKE THE SQUARE HITBOX BE FULLY INSIDE THE CIRCLE HITBOX
// CALCULATIONS USE THE SQUARE HITBOX DEFINED BY HAVING SIDES = R1*2, R2*2
struct ButtonPair
{
    double x1;
    double y1;
    double r1;
    double x2;
    double y2;
    double r2;
};

class Field
{
public:
    Field();
    Field(std::initializer_list<ButtonPair> button_pairs);

    void addButtonPair(ButtonPair button_pair);

    std::vector<ButtonPair> getButtonPairs();

    // Returns the index of the first button pair that is being pressed by the coupler head given by the positions
    // Returns -1 if no button is being pressed
    // USES A SQUARE HIT BOX TO INCREASE PERFORMANCE
    int getButtonPairPressedIndex(std::tuple<double, double> mech_pos_1, std::tuple<double, double> mech_pos_2);

private:
    std::vector<ButtonPair> ButtonPairs;
    bool isButtonPairPressed(std::tuple<double, double> mech_pos_1, std::tuple<double, double> mech_pos_2, ButtonPair button_pair);
};
#endif