#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include <tuple>
#include <random>
#include "FourBarMechanism.h"
#include "ctpl_stl.h"

// Defines the geometric limits for generating any mechanism
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

// This class handles the optimization of a generation of mechanisms using a genetic algorithm
class Optimizer
{
public:
    Optimizer(int generation_size, int chunk_size, int max_num_threads, std::function<double(FourBarMechanism)> fitness_function, GenerationLimits generation_limits);
    // Will optimize the generation for the given number of iterations
    void optimize(int iterations);

    // Will return the best mechanisms from the current generation
    std::vector<FourBarMechanism> getBestMechanisms(int num_mechanisms);
    FourBarMechanism getBestMechanism();
    // Simple setting functions
    void setLinearDensity(double linear_density);
    void setMutationRate(double mutation_rate);
    void setSurvivalRate(double survival_rate);

private:
    // Will split the generation into chunks and evaluate them in parallel
    // Returns a vector of tuples containing the mechanism and its fitness
    std::vector<std::tuple<FourBarMechanism, double>> evaluate_mechanisms(const std::vector<FourBarMechanism> &mechanisms);

    // Will select the best mechanisms from the evaluated mechanisms
    // The number of mechanisms returned will be defined by the survival rate
    // The best mechanisms will be copied to the next generation
    // The smaller the fitness, the better the mechanism
    std::vector<FourBarMechanism> select_mechanisms(const std::vector<std::tuple<FourBarMechanism, double>> &evaluated_mechanisms);

    // Will generate children from the best mechanisms by crossing over the parents with added mutations
    FourBarMechanism generate_children(const FourBarMechanism &parent1, const FourBarMechanism &parent2);

    // Will generate a random mechanism within the generation limits
    FourBarMechanism generate_random_mechanism();

    // Will generate a random mechanism within the generation limits
    std::vector<FourBarMechanism> generate_random_chunk(int chunk_size);

    // Random helping functions with speed optimization
    double random_double(double lower_limit, double upper_limit);
    int random_int(int lower_limit, int upper_limit);

    int keep_in_bounds(int value, int lower_limit, int upper_limit);

    // Will generate children from parent mechanisms
    std::vector<FourBarMechanism> generate_children_chunk(const std::vector<FourBarMechanism> &parents, int chunk_size);

    // At any moment will contain the current generation of selected mechanisms
    std::vector<FourBarMechanism> current_best_generation;

    // The geometric limits for generating any mechanism
    GenerationLimits generation_limits;

    // The fitness function to be optimized
    // The smaller the better
    std::function<double(FourBarMechanism)> fitness_function;

    // Configs
    int generation_size;
    int chunk_size;
    int num_threads;
    double linear_density = 1;
    double mutation_rate = 1;
    double survival_rate = 0.1;

    // Thread pool for parallel evaluation
    ctpl::thread_pool thread_pool;

    // Random engine for speed optimization
    std::random_device random_device;
    std::mt19937 random_engine;
    std::uniform_real_distribution<> uniform_dist;
};
#endif