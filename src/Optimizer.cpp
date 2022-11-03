#include "Optimizer.h"
#include "Link.h"
#include "CouplerHead.h"
#include <random>
#include <tuple>
#include <algorithm>
#include <iostream>

std::uniform_real_distribution<> dist(0, 1);

Optimizer::Optimizer(int generation_size, int chunk_size, int max_num_threads, std::function<double(FourBarMechanism)> fitness_function, GenerationLimits generation_limits)
{
    this->generation_size = generation_size;
    this->chunk_size = chunk_size;
    this->num_threads = max_num_threads;
    this->fitness_function = fitness_function;
    this->generation_limits = generation_limits;
    this->thread_pool.resize(num_threads);
    this->random_engine = std::mt19937(random_device());
    std::uniform_real_distribution<> uniform_dist(0, 1);
}

void Optimizer::setLinearDensity(double linear_density)
{
    this->linear_density = linear_density;
}

std::vector<FourBarMechanism> Optimizer::generate_random_chunk(int chunk_size)
{
    std::vector<FourBarMechanism> mechanisms;
    for (int i = 0; i < chunk_size; i++)
    {
        mechanisms.push_back(generate_random_mechanism());
    }
    return mechanisms;
}

FourBarMechanism Optimizer::generate_random_mechanism()
{
    std::tuple<double, double> input_ground_point = std::make_tuple(random_double(std::get<0>(generation_limits.input_ground_point_lower_limit), std::get<0>(generation_limits.input_ground_point_upper_limit)),
                                                                    random_double(std::get<1>(generation_limits.input_ground_point_lower_limit), std::get<1>(generation_limits.input_ground_point_upper_limit)));
    std::tuple<double, double> input_coupler_point = std::make_tuple(random_double(std::get<0>(generation_limits.input_coupler_point_lower_limit), std::get<0>(generation_limits.input_coupler_point_upper_limit)),
                                                                     random_double(std::get<1>(generation_limits.input_coupler_point_lower_limit), std::get<1>(generation_limits.input_coupler_point_upper_limit)));
    std::tuple<double, double> coupler_output_point = std::make_tuple(random_double(std::get<0>(generation_limits.coupler_output_point_lower_limit), std::get<0>(generation_limits.coupler_output_point_upper_limit)),
                                                                      random_double(std::get<1>(generation_limits.coupler_output_point_lower_limit), std::get<1>(generation_limits.coupler_output_point_upper_limit)));
    std::tuple<double, double> output_ground_point = std::make_tuple(random_double(std::get<0>(generation_limits.output_ground_point_lower_limit), std::get<0>(generation_limits.output_ground_point_upper_limit)),
                                                                     random_double(std::get<1>(generation_limits.output_ground_point_lower_limit), std::get<1>(generation_limits.output_ground_point_upper_limit)));
    std::tuple<double, double> couplertop_input_point = std::make_tuple(random_double(std::get<0>(generation_limits.couplertop_input_point_lower_limit), std::get<0>(generation_limits.couplertop_input_point_upper_limit)),
                                                                        random_double(std::get<1>(generation_limits.couplertop_input_point_lower_limit), std::get<1>(generation_limits.couplertop_input_point_upper_limit)));
    std::tuple<double, double> couplertop_output_point = std::make_tuple(random_double(std::get<0>(generation_limits.couplertop_output_point_lower_limit), std::get<0>(generation_limits.couplertop_output_point_upper_limit)),
                                                                         random_double(std::get<1>(generation_limits.couplertop_output_point_lower_limit), std::get<1>(generation_limits.couplertop_output_point_upper_limit)));
    Link input_link = Link(input_ground_point, input_coupler_point, linear_density);
    Link coupler_link = Link(input_coupler_point, coupler_output_point, linear_density);
    Link output_link = Link(coupler_output_point, output_ground_point, linear_density);
    CouplerHead coupler_head = CouplerHead(input_link, output_link, couplertop_input_point, couplertop_output_point, linear_density);
    FourBarMechanism mechanism = FourBarMechanism(input_link, coupler_link, output_link, coupler_head);
    // Setting for default, pi/2 angle
    mechanism.rotate(1.57079632679, 1);
    return mechanism;
}

FourBarMechanism Optimizer::generate_children(const FourBarMechanism &parent1, const FourBarMechanism &parent2)
{
    auto [parent1_input_ground_point, parent1_input_coupler_point] = parent1.getInputLinkPositions();
    auto [parent1_coupler_output_point, parent1_output_ground_point] = parent1.getOutputLinkPositions();
    auto [parent1_couplertop_input_point, parent1_couplertop_output_point] = parent1.getCouplerHeadTopPositions();

    auto [parent2_input_ground_point, parent2_input_coupler_point] = parent2.getInputLinkPositions();
    auto [parent2_coupler_output_point, parent2_output_ground_point] = parent2.getOutputLinkPositions();
    auto [parent2_couplertop_input_point, parent2_couplertop_output_point] = parent2.getCouplerHeadTopPositions();

    std::tuple<double, double> input_ground_point = std::make_tuple(random_double(std::get<0>(parent1_input_ground_point), std::get<0>(parent2_input_ground_point)) * (1 + random_double(-1, 1) * mutation_rate),
                                                                    random_double(std::get<1>(parent1_input_ground_point), std::get<1>(parent2_input_ground_point)) * (1 + random_double(-1, 1) * mutation_rate));
    std::tuple<double, double> input_coupler_point = std::make_tuple(random_double(std::get<0>(parent1_input_coupler_point), std::get<0>(parent2_input_coupler_point)) * (1 + random_double(-1, 1) * mutation_rate),
                                                                     random_double(std::get<1>(parent1_input_coupler_point), std::get<1>(parent2_input_coupler_point)) * (1 + random_double(-1, 1) * mutation_rate));
    std::tuple<double, double> coupler_output_point = std::make_tuple(random_double(std::get<0>(parent1_coupler_output_point), std::get<0>(parent2_coupler_output_point)) * (1 + random_double(-1, 1) * mutation_rate),
                                                                      random_double(std::get<1>(parent1_coupler_output_point), std::get<1>(parent2_coupler_output_point)) * (1 + random_double(-1, 1) * mutation_rate));
    std::tuple<double, double> output_ground_point = std::make_tuple(random_double(std::get<0>(parent1_output_ground_point), std::get<0>(parent2_output_ground_point)) * (1 + random_double(-1, 1) * mutation_rate),
                                                                     random_double(std::get<1>(parent1_output_ground_point), std::get<1>(parent2_output_ground_point)) * (1 + random_double(-1, 1) * mutation_rate));
    std::tuple<double, double> couplertop_input_point = std::make_tuple(random_double(std::get<0>(parent1_couplertop_input_point), std::get<0>(parent2_couplertop_input_point)) * (1 + random_double(-1, 1) * mutation_rate),
                                                                        random_double(std::get<1>(parent1_couplertop_input_point), std::get<1>(parent2_couplertop_input_point)) * (1 + random_double(-1, 1) * mutation_rate));
    std::tuple<double, double> couplertop_output_point = std::make_tuple(random_double(std::get<0>(parent1_couplertop_output_point), std::get<0>(parent2_couplertop_output_point)) * (1 + random_double(-1, 1) * mutation_rate),
                                                                         random_double(std::get<1>(parent1_couplertop_output_point), std::get<1>(parent2_couplertop_output_point)) * (1 + random_double(-1, 1) * mutation_rate));
    Link input_link = Link(input_ground_point, input_coupler_point, linear_density);
    Link coupler_link = Link(input_coupler_point, coupler_output_point, linear_density);
    Link output_link = Link(coupler_output_point, output_ground_point, linear_density);
    CouplerHead coupler_head = CouplerHead(input_link, output_link, couplertop_input_point, couplertop_output_point, linear_density);
    FourBarMechanism mechanism = FourBarMechanism(input_link, coupler_link, output_link, coupler_head);
    mechanism.rotate(1.57079632679, 1);
    return mechanism;
}

std::vector<FourBarMechanism> Optimizer::generate_children_chunk(const std::vector<FourBarMechanism> &parents, int chunk_size)
{
    std::vector<FourBarMechanism> children;
    for (int i = 0; i < chunk_size; i++)
    {
        int parent1_index = random_int(0, parents.size() - 1);
        int parent2_index = random_int(0, parents.size() - 1);
        children.push_back(generate_children(parents[parent1_index], parents[parent2_index]));
    }
    return children;
}

int Optimizer::random_int(int lower_limit, int upper_limit)
{
    return keep_in_bounds(lower_limit + uniform_dist(random_engine) * (upper_limit - lower_limit), lower_limit, upper_limit);
}

int keep_in_bounds(int value, int lower_limit, int upper_limit)
{
    if (value < lower_limit)
    {
        return lower_limit;
    }
    else if (value > upper_limit)
    {
        return upper_limit;
    }
    else
    {
        return value;
    }
}
double Optimizer::random_double(double lower_limit, double upper_limit)
{
    return lower_limit + (upper_limit - lower_limit) * uniform_dist(random_engine);
}

static std::vector<std::tuple<FourBarMechanism, double>> evaluate_chunk(int id, const std::vector<FourBarMechanism> &mechanisms, std::function<double(FourBarMechanism)> fitness_function)
{
    std::vector<std::tuple<FourBarMechanism, double>> evaluated_mechanisms;
    int chunk_size = mechanisms.size();
    for (int i = 0; i < chunk_size; i++)
    {
        evaluated_mechanisms.push_back(std::make_tuple(mechanisms[i], fitness_function(mechanisms[i])));
    }
    return evaluated_mechanisms;
}

std::vector<std::tuple<FourBarMechanism, double>> Optimizer::evaluate_mechanisms(const std::vector<FourBarMechanism> &mechanisms)
{
    std::vector<std::tuple<FourBarMechanism, double>> evaluated_mechanisms;
    std::vector<std::future<std::vector<std::tuple<FourBarMechanism, double>>>> futures;
    for (int i = 0; i < mechanisms.size(); i += chunk_size)
    {
        // Standard library version
        // futures.push_back(std::async(std::launch::async, evaluate_chunk, std::vector<FourBarMechanism>(mechanisms.begin() + i, mechanisms.begin() + i + chunk_size), chunk_size, this->fitness_function));
        // Thread pool version
        futures.push_back(thread_pool.push(evaluate_chunk, std::vector<FourBarMechanism>(mechanisms.begin() + i, mechanisms.begin() + i + chunk_size), this->fitness_function));
    }
    // Waits for all threads to finish, including non-executed functions.
    thread_pool.stop(true);
    for (int i = 0; i < futures.size(); i++)
    {
        std::vector<std::tuple<FourBarMechanism, double>> evaluated_mechanisms_chunk = futures[i].get();
        evaluated_mechanisms.insert(evaluated_mechanisms.end(), evaluated_mechanisms_chunk.begin(), evaluated_mechanisms_chunk.end());
    }
    return evaluated_mechanisms;
}

std::vector<FourBarMechanism> Optimizer::select_mechanisms(const std::vector<std::tuple<FourBarMechanism, double>> &evaluated_mechanisms)
{
    std::vector<FourBarMechanism> selected_mechanisms;
    std::vector<double> fitnesses;
    int population_size = std::max(1, (int)(evaluated_mechanisms.size() * survival_rate));
    for (int i = 0; i < evaluated_mechanisms.size(); i++)
    {
        fitnesses.push_back(std::get<1>(evaluated_mechanisms[i]));
    }
    std::sort(fitnesses.begin(), fitnesses.end());
    double fitness_threshold = fitnesses[population_size];
    for (int i = 0; i < evaluated_mechanisms.size(); i++)
    {
        if (std::get<1>(evaluated_mechanisms[i]) <= fitness_threshold)
        {
            selected_mechanisms.push_back(std::get<0>(evaluated_mechanisms[i]));
        }
    }
    return selected_mechanisms;
}

void Optimizer::optimize(int iterations)
{
    std::vector<FourBarMechanism> current_gen = generate_random_chunk(generation_size);
    for (int i = 0; i < iterations; i++)
    {
        std::cout << "Generation " << i << " of " << iterations << " completed."
                  << "\n";
        // Evaluates the current generation
        std::vector<FourBarMechanism> selected_mechanisms = select_mechanisms(evaluate_mechanisms(current_gen));
        // Replaces it with the next generation
        current_gen = generate_children_chunk(selected_mechanisms, generation_size);
    }
    this->current_best_generation = select_mechanisms(evaluate_mechanisms(current_gen));
}

std::vector<FourBarMechanism> Optimizer::getBestMechanisms(int num_mechanisms)
{
    return current_best_generation;
}

FourBarMechanism Optimizer::getBestMechanism()
{
    std::vector<std::tuple<FourBarMechanism, double>> evaluations = evaluate_mechanisms(current_best_generation);
    std::vector<double> fitnesses;
    for (int i = 0; i < evaluations.size(); i++)
    {
        fitnesses.push_back(std::get<1>(evaluations[i]));
    }
    int best_index = std::distance(fitnesses.begin(), std::max_element(fitnesses.begin(), fitnesses.end()));
    return std::get<0>(evaluations[best_index]);
}