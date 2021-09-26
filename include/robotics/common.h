#pragma once

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
#define _USE_MATH_DEFINES
#endif

#include <Eigen/Dense>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>

namespace Robotics {
    template<int N>
    using ColumnVector = Eigen::Matrix<double, N, 1>;

    template<int N>
    using SquareMatrix = Eigen::Matrix<double, N, N>;

    template<int N, int M>
    using Matrix = Eigen::Matrix<double, N, M>;

    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    class NormalDistributionRandomGenerator {
      public:

        template <int Size>
        ColumnVector<Size> GetColumnVector() {
            ColumnVector<Size> rand_vector;

            for (auto& element : vector)
                element = distribution(generator);
            
            return rand_vector;
        }
        
      private:
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(0.0, 1.0);
    }

}  // namespace Robotics