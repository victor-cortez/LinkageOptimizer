#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include <tuple>
#include "FourBarMechanism.h"
#include "ctpl_stl.h"
#include <random>

struct GenerationLimits
{
    std::tuple<double, double> input_ground_point_lower_limit;
    std::tuple<double, double> input_ground_point_upper_limit;
    std::tuple<double, double> input_coupler_point_lower_limit;
    std::tuple<double, double> input_coupler_point_upper_limit;
    std::tuple<double, double> coupler_output_point_lower_limit;
    std::tuple<double, double> coupler_output_point_upper_limit;
    std::tuple<double, double> output_ground_point_lower_limit;
    std::tuple<double, double> output_ground_point_upper_limit;
    std::tuple<double, double> couplertop_input_point_lower_limit;
    std::tuple<double, double> couplertop_input_point_upper_limit;
    std::tuple<double, double> couplertop_output_point_lower_limit;
    std::tuple<double, double> couplertop_output_point_upper_limit;
};

class Optimizer
{
public:
    Optimizer(int generation_size, int chunk_size, int num_threads, double (*fitness_function)(FourBarMechanism), GenerationLimits generation_limits);
    void run(int num_generations);
    std::vector<std::tuple<FourBarMechanism, double>> getBestMechanisms(int num_mechanisms);
    void setLinearDensity(double linear_density);
    void setMutationRate(double mutation_rate);

private:
    FourBarMechanism generate_children(const FourBarMechanism &parent1, const FourBarMechanism &parent2);
    FourBarMechanism generate_random_mechanism();
    double random_double(double lower_limit, double upper_limit);
    int random_int(int lower_limit, int upper_limit);
    std::vector<FourBarMechanism> generate_random_chunk(int chunk_size);
    std::vector<FourBarMechanism> generate_children_chunk(const std::vector<FourBarMechanism> &parents, int chunk_size);
    std::vector<FourBarMechanism> current_generation;
    std::vector<FourBarMechanism> next_generation;
    GenerationLimits generation_limits;
    double (*fitness_function)(FourBarMechanism);
    int generation_size;
    int chunk_size;
    int num_threads;
    ctpl::thread_pool thread_pool;
    std::random_device random_device;
    std::mt19937 random_engine;
    std::uniform_real_distribution<> uniform_dist;
    double linear_density = 1;
    double mutation_rate = 1;
};
#endif