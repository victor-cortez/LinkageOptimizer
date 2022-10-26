#include "Field.h"

void Field::addButtonPair(ButtonPair button_pair)
{
    this->ButtonPairs.push_back(button_pair);
}

std::vector<ButtonPair> Field::getButtonPairs()
{
    return this->ButtonPairs;
}

// Returns the index of the first button pair that is being pressed by the coupler head given by the positions
// Returns -1 if no button is being pressed
// USES A SQUARE HIT BOX TO INCREASE PERFORMANCE
int Field::getButtonPairPressedIndex(std::tuple<double, double> mech_pos_1, std::tuple<double, double> mech_pos_2)
{
    int size = this->ButtonPairs.size();
    for (int i = 0; i < size; i++)
    {
        if (isButtonPairPressed(mech_pos_1, mech_pos_2, this->ButtonPairs[i]))
        {
            return i;
        }
    }
    return -1;
}

// USES A SQUARE HIT BOX TO INCREASE PERFORMANCE
bool Field::isButtonPairPressed(std::tuple<double, double> mech_pos_1, std::tuple<double, double> mech_pos_2, ButtonPair button_pair)
{
    auto [x1, y1] = mech_pos_1;
    auto [x2, y2] = mech_pos_2;
    double x1b = button_pair.x1;
    double y1b = button_pair.y1;
    double r1b = button_pair.r1;
    double x2b = button_pair.x2;
    double y2b = button_pair.y2;
    double r2b = button_pair.r2;
    if (abs(x1 - x1b) < r1b && abs(y1 - y1b) < r1b && abs(x2 - x2b) < r2b && abs(y2 - y2b) < r2b)
    {
        return true;
    }
    return false;
}