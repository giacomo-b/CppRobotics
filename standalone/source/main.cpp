#include <robotics/robotics.h>

#include <Eigen/Dense>
#include <iostream>
#include <random>

using Robotics::Coordinates;

int main()
{
    const int n_test = 10;
    const double area = 100.0;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-area, area);

    double dt = 0.1;

    Eigen::MatrixXd A(2, 2);
    A << dt, 1.0, 0, dt;

    Eigen::VectorXd B(2);
    B << 0, 1;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    Robotics::PathPlanning::LQR lqr_planner(A, B, Q, R);
    lqr_planner.setTimeStep(dt);

    for (auto _ = n_test; _--;) {
        Coordinates initial{.x = 6.0, .y = 6.0};
        Coordinates target{.x = distribution(generator), .y = distribution(generator)};

        std::vector<Coordinates> path = lqr_planner.Solve(initial, target);

        std::cout << "Goal: " << target.x << '\t' << target.y << '\n';
        std::cout << path << std::endl;
    }

    return 0;
}
