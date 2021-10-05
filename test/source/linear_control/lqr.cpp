#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <robotics/linear_control/lqr.hpp>
#include <string>

TEST_CASE("Robotics")
{
    static constexpr int N = 2;
    static constexpr int M = 1;

    const double dt = 0.1;

    // State matrix
    Robotics::SquareMatrix<N> A;
    A << dt, 1.0, 0, dt;

    // Control matrix
    Robotics::Matrix<N, M> B;
    B << 0, 1;

    // Weights
    Robotics::SquareMatrix<N> Q = Robotics::SquareMatrix<N>::Identity();
    Robotics::SquareMatrix<M> R = Robotics::SquareMatrix<M>::Identity();

    Robotics::LinearControl::LQR<N, M> planner(A, B, Q, R);

    CHECK(true == true);
}