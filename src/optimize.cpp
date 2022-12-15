#include <iostream>
#include <fstream>
#include <chrono>
#include <numeric>
#include "Link.h"
#include "FourBarMechanism.h"
#include "CouplerHead.h"
#include "Field.h"
#include "Optimizer.h"

constexpr double std_mass_linear_density = 1.0; // Kg/m
constexpr double button_radius = 0.0142;
constexpr double hitbox_radius = button_radius / 1.41421356237; // radius / sqrt(2)
constexpr double PI = 3.14159265358979323846;

double fitnessFunction(FourBarMechanism mechanism)
{
    Field playing_field({ButtonPair{0.1143, 0.3429, hitbox_radius, 0.163322, 0.329692, hitbox_radius}, ButtonPair{0.254, 0.381, hitbox_radius, 0.3048, 0.381, hitbox_radius}, ButtonPair{0.408686, 0.315214, hitbox_radius, 0.4445, 0.2794, hitbox_radius}});
    double angle = 0;
    double fitness = 0;
    double angle_step = 0.001;
    double dt = 0.01;
    bool was_button_pressed[3] = {false, false, false};
    std::vector<double> energies;
    long count = 0;
    std::cout << "Fitness function called" << std::endl;
    while (angle < 2 * PI)
    {
        try
        {
            mechanism.rotate(angle, dt);
            for (int i = 0; i < 3; i++)
            {
                auto [pos1, pos2] = mechanism.getCouplerHeadTopPositions();
                int button_index = playing_field.getButtonPairPressedIndex(pos1, pos2);
                if (button_index != -1)
                {
                    was_button_pressed[button_index] = true;
                }
            }
            if (count < 2)
            {
                double energy = mechanism.getTotalEnergy();
                if (!std::isnan(energy) && !std::isnan(-energy))
                {
                    energies.push_back(energy);
                }
            }
        }
        catch (...)
        {
        }
        angle += angle_step;
        count++;
    }
    double energy_mean = std::accumulate(energies.begin(), energies.end(), 0.0) / energies.size();

    for (auto energy : energies)
    {
        std::cout << energy << ",";
    }
    std::cout << std::endl;
    std::vector<double> deviation_from_the_mean;
    for (auto energy : energies)
    {
        deviation_from_the_mean.push_back(std::abs(energy - energy_mean));
    }
    double average_deviation_from_the_mean = std::accumulate(deviation_from_the_mean.begin(), deviation_from_the_mean.end(), 0.0) / deviation_from_the_mean.size();
    if (!std::isnan(average_deviation_from_the_mean))
    {
        fitness += 1 * average_deviation_from_the_mean;
    }
    fitness -= energies.size() * 1000;
    /*
    if (was_button_pressed[0] || was_button_pressed[1] || was_button_pressed[2] == false)
    {
        fitness = std::numeric_limits<double>::infinity();
    }
    */
    for (int i = 0; i < 3; i++)
    {
        if (was_button_pressed[i] == false)
        {
            fitness += 1000;
        }
    }
    std::cout << "Fitness: " << fitness << std::endl;
    return fitness;
}

int main()
{
    GenerationLimits limits;
    limits.input_ground_point_lower_limit = std::make_tuple(0.0, 0.0);
    limits.input_ground_point_upper_limit = std::make_tuple(0.3048, 0.3048);
    limits.input_coupler_point_lower_limit = std::make_tuple(0.0, 0.0);
    limits.input_coupler_point_upper_limit = std::make_tuple(0.6096, 0.6096);
    limits.coupler_output_point_lower_limit = std::make_tuple(0.0, 0.0);
    limits.coupler_output_point_upper_limit = std::make_tuple(0.6096, 0.6096);
    limits.output_ground_point_lower_limit = std::make_tuple(0.0, 0.0);
    limits.output_ground_point_upper_limit = std::make_tuple(0.3048, 0.3048);
    limits.couplertop_input_point_lower_limit = std::make_tuple(0.0, 0.0);
    limits.couplertop_input_point_upper_limit = std::make_tuple(0.6096, 0.6096);
    limits.couplertop_output_point_lower_limit = std::make_tuple(0.0, 0.0);
    limits.couplertop_output_point_upper_limit = std::make_tuple(0.6096, 0.6096);

    int generation_size = 100;
    int chunk_size = 10;
    int max_num_threads = 4;
    int num_generations = 100;

    Optimizer optimizer(generation_size, chunk_size, max_num_threads, fitnessFunction, limits);
    auto start = std::chrono::system_clock::now();
    optimizer.optimize(num_generations);
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Time taken: " << elapsed.count() << " microseconds" << std::endl;
}