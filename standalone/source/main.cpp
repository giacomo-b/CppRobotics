#include <robotics/robotics.h>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <random>

using Robotics::Coordinates;

int main()
{
    const double dt = 0.1;

    // State matrix
    Eigen::MatrixXd A(2, 2);
    A << dt, 1.0, 0, dt;

    // Control matrix
    Eigen::VectorXd B(2);
    B << 0, 1;

    // Weights
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    Coordinates initial{.x = 6.0, .y = 6.0};

    Robotics::PathPlanning::LQR lqr_planner(A, B, Q, R);
    lqr_planner.setTimeStep(dt);

    const int n_test = 10;
    const double area = 100.0;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-area, area);

    for (auto _ = n_test; _--;) {
        Coordinates target{.x = distribution(generator), .y = distribution(generator)};

        std::cout << "Goal: (" << target.x << ", " << target.y << "), ";
        auto t_start = std::chrono::high_resolution_clock::now();

        std::vector<Coordinates> path = lqr_planner.Solve(initial, target);

        auto t_end = std::chrono::high_resolution_clock::now();
        if (!path.empty()) {
            std::cout << "solution found in "
                      << std::chrono::duration<double, std::milli>(t_end - t_start).count()
                      << " ms.\n";
        }

        std::cout << path << std::endl;
    }

    return 0;
}
