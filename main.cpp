#include <iostream>
#include <fstream>
#include "Link.h"
#include "FourBarMechanism.h"
#include "CouplerHead.h"
#include <chrono>

int main()
{
    auto start = std::chrono::steady_clock::now();
    Link crank_link = Link(std::make_tuple(0.0, 0.0), std::make_tuple(0.0, 1.0), 0, 0);
    Link coupler_link = Link(std::make_tuple(0.0, 1.0), std::make_tuple(0.2, 1.0), 0, 0);
    Link output_link = Link(std::make_tuple(0.2, 1.0), std::make_tuple(1.0, 0.2), 0, 0);
    CouplerHead header_link = CouplerHead(crank_link, output_link, std::make_tuple(0.0, 0.3), std::make_tuple(0.3, 0.7));
    FourBarMechanism mechanism = FourBarMechanism(crank_link, coupler_link, output_link, header_link);

    std::ofstream myfile("example.txt");
    double min_angle = 3.1415 / 2 - 0.2;
    double max_angle = 3.1415 / 2 + 0.2;
    double angle_step = 0.001;
    double angle = min_angle;
    if (myfile.is_open())
    {
        try
        {
            myfile << mechanism.getDumpHeader();
            while (angle < max_angle)
            {
                mechanism.rotate(angle, 0.01);
                myfile << mechanism.dumpState();
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