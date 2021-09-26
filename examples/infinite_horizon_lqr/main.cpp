#include <robotics/robotics.h>

#include <chrono>
#include <iostream>
#include <random>

int main()
{
    static constexpr int N = 2;
    static constexpr int M = 1;

    const double dt = 0.1;

    // State matrix
    Robotics::SquareMatrix<N> A;
    A << dt, 1.0, 0, dt;

    // Control matrix
    Robotics::Matrix<N,M> B;
    B << 0, 1;

    // Weights
    Robotics::SquareMatrix<N> Q = Robotics::SquareMatrix<N>::Identity();
    Robotics::SquareMatrix<M> R = Robotics::SquareMatrix<M>::Identity();

    Robotics::LinearControl::LQR<N,M> lqr_planner(A, B, Q, R);
    lqr_planner.SetTimeStep(dt);

    const int n_test = 10;
    const double area = 100.0;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-area, area);

    Robotics::ColumnVector<N> start = {6.0, 6.0};

    for (auto _ = n_test; _--;) {
        Robotics::ColumnVector<N> target;
        target << distribution(generator), distribution(generator);

        std::cout << "Goal: (" << target(0) << ", " << target(1) << "), ";
        auto t_start = std::chrono::high_resolution_clock::now();

        auto path = lqr_planner.Solve(start, target);

        auto t_end = std::chrono::high_resolution_clock::now();
        if (!path.empty()) {
            std::cout << "solution found in "
                      << std::chrono::duration<double, std::milli>(t_end - t_start).count()
                      << " ms.\n";

            for (const auto& point : path)
                std::cout << std::fixed << std::setprecision(4) << point.transpose() << '\n';
        }
    }

    return 0;
}
