#include <iostream>
#include <tuple>
#include <cmath>

constexpr double det(const double matrix[3][3])
{
    return matrix[0][0] * matrix[1][1] * matrix[2][2] + matrix[0][1] * matrix[1][2] * matrix[2][0] + matrix[0][2] * matrix[1][0] * matrix[2][1] - matrix[0][2] * matrix[1][1] * matrix[2][0] - matrix[0][1] * matrix[1][0] * matrix[2][2] - matrix[0][0] * matrix[1][2] * matrix[2][1];
}

class CouplerRootPointsAreColinear : public std::exception
{
private:
    char *message;

public:
    CouplerRootPointsAreColinear(char *msg) : message(msg) {}
    char *what()
    {
        return message;
    }
};

constexpr std::tuple<double, double> getCircleCenter(const std::tuple<double, double> a1, const std::tuple<double, double> a2, const std::tuple<double, double> a3)
{
    // We know that the distance from our point x,y must be equal to r in each of the 3 points
    // The formula of the circle is
    // x^2 + y^2 + ax + by + c = 0
    // But the same formula based on A,B root points and radius is
    // (x-A)^2 + (y-B)^2 = R^2
    // x^2 - 2Ax + A^2 + y^2 - 2By + B^2 = R^2
    // x^2 + y^2 + (-2A)x + (-2BY) + (A^2  + B^2  - R^2) = 0
    // With a = -2A, b = -2B, c = A^2 + B^2 - R^2
    // Our Equations are therefore
    // a*a1 + b*b1 + c = -(a1^2+b1^2)
    // a*a2 + b*b2 + c = -(a2^2+b2^2)
    // a*a3 + b*b3 + c = -(a3^2+b3^2)

    // Given our 3 results d1,d2,d3
    double d1 = -(std::pow(std::get<0>(a1), 2) + std::pow(std::get<1>(a1), 2));
    double d2 = -(std::pow(std::get<0>(a2), 2) + std::pow(std::get<1>(a2), 2));
    double d3 = -(std::pow(std::get<0>(a3), 2) + std::pow(std::get<1>(a3), 2));

    // Our Cramer's rule matrices are
    double D[3][3] = {
        {std::get<0>(a1), std::get<1>(a1), 1},
        {std::get<0>(a2), std::get<1>(a2), 1},
        {std::get<0>(a3), std::get<1>(a3), 1}};
    double Dx[3][3] = {
        {d1, std::get<1>(a1), 1},
        {d2, std::get<1>(a2), 1},
        {d3, std::get<1>(a3), 1}};
    double Dy[3][3] = {
        {std::get<0>(a1), d1, 1},
        {std::get<0>(a2), d2, 1},
        {std::get<0>(a3), d3, 1}};

    // Now we calculate our coefficients
    double detD = det(D);
    if (detD == 0)
    {
        throw CouplerRootPointsAreColinear("The coupler root points are colinear. Division by Zero error iminent!");
    }
    double detDx = det(Dx);
    double detDy = det(Dy);

    double a = detDx / detD;
    double b = detDy / detD;

    double x = -a / 2;
    double y = -b / 2;
    return std::make_tuple(x, y);
}

constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> getLinkPositionsFromCouplers(
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_1,
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_2,
    const std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_3)
{
    const std::tuple<double, double> a1 = std::get<0>(coupler_pos_1);
    const std::tuple<double, double> b1 = std::get<1>(coupler_pos_1);
    const std::tuple<double, double> a2 = std::get<0>(coupler_pos_2);
    const std::tuple<double, double> b2 = std::get<1>(coupler_pos_2);
    const std::tuple<double, double> a3 = std::get<0>(coupler_pos_3);
    const std::tuple<double, double> b3 = std::get<1>(coupler_pos_3);

    const std::tuple<double, double> crank_link_root = getCircleCenter(a1, a2, a3);
    const std::tuple<double, double> output_link_root = getCircleCenter(b1, b2, b3);

    return std::make_tuple(crank_link_root, output_link_root);
}

int main()
{
    constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_1 =
        std::make_tuple(std::make_tuple(60.0, 223), std::make_tuple(112, 223));
    constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_2 =
        std::make_tuple(std::make_tuple(178.292, 227.1984), std::make_tuple(225.42, 205.213));
    constexpr std::tuple<std::tuple<double, double>, std::tuple<double, double>> coupler_pos_3 =
        std::make_tuple(std::make_tuple(247.96, 154.9371), std::make_tuple(238.0338, 103.89));

    const auto [crank_link_root_pos, output_link_root_pos] =
        getLinkPositionsFromCouplers(coupler_pos_1, coupler_pos_2, coupler_pos_3);
    std::cout << "crank_link_root_pos: (" << std::get<0>(crank_link_root_pos) << ", " << std::get<1>(crank_link_root_pos) << ")\n";
    std::cout << "output_link_root_pos: (" << std::get<0>(output_link_root_pos) << ", " << std::get<1>(output_link_root_pos) << ")\n";
}