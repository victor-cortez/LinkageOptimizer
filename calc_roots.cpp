#include <iostream>
#include "FourBarMechanism.h"
#include <chrono>
#include <tuple>

int main()
{
    auto start = std::chrono::steady_clock::now();
    constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_1 =
        std::make_tuple(std::make_tuple(0.0, 0.0), std::make_tuple(0.0, 1.0));
    constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_2 =
        std::make_tuple(std::make_tuple(0.1, 1.0), std::make_tuple(0.2, 1.0));
    constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_3 =
        std::make_tuple(std::make_tuple(0.3, 1.0), std::make_tuple(0.4, 0.8));

    const auto [crank_link_root_pos, output_link_root_pos] =
        FourBarMechanism::getLinkPositionsFromCouplers(coupler_pos_1, coupler_pos_2, coupler_pos_3);
    std::cout << "crank_link_root_pos: " << std::get<0>(crank_link_root_pos) << "," << std::get<1>(crank_link_root_pos) << "\n";
    std::cout << "output_link_root_pos: " << std::get<0>(output_link_root_pos) << "," << std::get<1>(output_link_root_pos) << "\n";
    auto end = std::chrono::steady_clock::now();
    std::cout << "Time elapse: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
              << " microseconds\n";
}