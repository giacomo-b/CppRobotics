#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <random>
#include <vector>

namespace Robotics {
    template <int N>
    using ColumnVector = Eigen::Matrix<double, N, 1>;

    template <int N>
    using SquareMatrix = Eigen::Matrix<double, N, N>;

    template <int N, int M>
    using Matrix = Eigen::Matrix<double, N, M>;

    inline double deg2rad(double deg)
    {
        return deg * 3.14 / 180.0;  // TODO: replace 3.14 with M_PI (cross-platform)
    }

   /**
    * @brief return evenly spaced numbers over a specified N interval.
    * @param start The starting value of the sequence
    * @param stop The end value of the sequence
    * @param N number of sample to generate. 
    */
    template <typename T>
    inline constexpr std::vector<T> linspace(T start, T stop, std::size_t N)
    {
        assert(N > 1);
        std::vector<T> vec(N, 0);

        T val{start};
        T increment{(stop - start) / static_cast<T>(N - 1)};
        for (auto it = vec.begin(); it != vec.end(); ++it)
        {
            *it = val;
            val += increment;
        }
        return vec;
    }

    class NormalDistributionRandomGenerator {
      public:
        template <int Size>
        ColumnVector<Size> GetColumnVector()
        {
            ColumnVector<Size> rand_vector;

            for (auto i = 0; i < rand_vector.size(); i++) rand_vector(i) = distribution(generator);

            return rand_vector;
        }

      private:
        std::default_random_engine generator;
        std::normal_distribution<double> distribution{0.0, 1.0};
    };

}  // namespace Robotics
