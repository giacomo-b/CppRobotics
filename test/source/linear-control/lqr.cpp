#include <doctest/doctest.h>
#include <robotics/linear-control/lqr.h>
#include <Eigen/Dense>
#include <string>

TEST_CASE("Robotics")
{
    using namespace Robotics::LinearControl;

    const double dt = 0.1;

    Eigen::MatrixXd A(2, 2);
    A << dt, 1.0, 0, dt;

    Eigen::VectorXd B(2);
    B << 0, 1;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);

    LQR planner(A, B, Q, R);

    CHECK(true == true);
}