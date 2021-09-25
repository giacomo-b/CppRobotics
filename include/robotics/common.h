#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <vector>

namespace Robotics {
    template<int N>
    using ColumnVector = Eigen::Matrix<double, N, 1>;

    template<int N>
    using SquareMatrix = Eigen::Matrix<double, N, N>;

    template<int N, int M>
    using Matrix = Eigen::Matrix<double, N, M>;

}  // namespace Robotics