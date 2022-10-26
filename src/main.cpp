#include <iostream>
#include <fstream>
#include <chrono>
#include "Link.h"
#include "FourBarMechanism.h"
#include "CouplerHead.h"
#include "Field.h"

constexpr double std_mass_linear_density = 1.0; // Kg/m
constexpr double button_radius = 0.05;
constexpr double hitbox_radius = button_radius / 1.41421356237; // radius / sqrt(2)
int main()
{
    auto start = std::chrono::steady_clock::now();
    Link crank_link = Link(std::make_tuple(0.0, 0.0), std::make_tuple(0.0, 1.0), std_mass_linear_density);
    Link coupler_link = Link(std::make_tuple(0.0, 1.0), std::make_tuple(0.2, 1.0), std_mass_linear_density);
    Link output_link = Link(std::make_tuple(0.2, 1.0), std::make_tuple(1.0, 0.2), std_mass_linear_density);
    CouplerHead header_link = CouplerHead(crank_link, output_link, std::make_tuple(0.0, 0.3), std::make_tuple(0.3, 0.7), std_mass_linear_density);
    FourBarMechanism mechanism = FourBarMechanism(crank_link, coupler_link, output_link, header_link);

    Field playing_field;
    playing_field.addButtonPair({0, 0, hitbox_radius, 1, 1, hitbox_radius});
    playing_field.addButtonPair({0, 0, hitbox_radius, 1, 1, hitbox_radius});
    playing_field.addButtonPair({0, 0, hitbox_radius, 1, 1, hitbox_radius});

    std::ofstream myfile("example2ad.txt");
    double min_angle = 3.1415 / 2 - 0.2;
    double max_angle = 3.1415 / 2 + 0.2;
    double angle_step = 0.001;
    double angle = min_angle;
    if (myfile.is_open())
    {
        try
        {
            myfile << mechanism.getDumpHeader() << "\n";
            while (angle < max_angle)
            {
                mechanism.rotate(angle, 0.01);
                myfile << mechanism.dumpState() << "\n";
                angle += angle_step;
            }
        }
        catch (...)
        {
            std::cout << "Error in moving the mechanism\n";
        }

        myfile.close();
    }
    else
    {
        std::cout << "Unable to open file\n";
    }
    auto end = std::chrono::steady_clock::now();
    std::cout << "Time elapse: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000000.0
              << " seconds\n";
}