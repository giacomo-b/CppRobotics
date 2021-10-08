#ifndef CUBIC_SPLINE_INTERPOLATION_H
#define CUBIC_SPLINE_INTERPOLATION_H

#include <robotics/common.hpp>

namespace Robotics::Polynomial {
    struct SplineSet {
        /**
         * @brief Y_i(t) = a_i + b_i * t + c_i * t^2 + d_i * t^3
         */
        double a{0.0};
        double b{0.0};
        double c{0.0};
        double d{0.0};
        SplineSet(const double& a, const double& b, const double& c, const double& d)
            : a(a), b(b), c(c), d(d)
        {
        }
    };

    /**
     * @brief A class for implementing a 1D cubic spline interpolation
     */
    class CubicSpline {
      public:
        CubicSpline() = default;
        CubicSpline(const std::vector<double>& x, const std::vector<double>& y,
                    const std::size_t& interpolation_size);
        void computeAllConstants();

        // Getter
        std::vector<double> getdx() { return m_dx; }
        std::vector<double> geta() { return m_a; }
        std::vector<double> getb() { return m_b; }
        std::vector<double> getc() { return m_c; }
        std::vector<double> getd() { return m_d; }
        std::vector<SplineSet> getSS() { return m_spline; }
        std::vector<double> getInterval() { return m_interval; }
        std::vector<double> getPosition() { return m_position; }
        std::vector<double> getFirstDerivative() { return m_first_derivative; }
        std::vector<double> getSecondDerivative() { return m_second_derivative; }

      private:
        std::size_t n_points{0};  // number of points
        std::vector<double> m_x{};
        std::vector<double> m_y{};
        std::vector<SplineSet> m_spline{};
        std::vector<double> m_dx{};
        std::vector<double> m_dy{};
        std::vector<double> m_dy_dx{};
        std::vector<double> m_a{};
        std::vector<double> m_b{};
        std::vector<double> m_c{};
        std::vector<double> m_d{};
        std::vector<double> m_interval{};
        std::vector<SplineSet> m_val{};
        std::vector<double> m_position{};
        std::vector<double> m_first_derivative{};
        std::vector<double> m_second_derivative{};
        std::vector<double> m_third_dev{};

        /**
         * @brief Computes the adjacent differences between successive elements in a waypoint
         * @param input waypoint set
         * @param output waypoint set
         */
        void adjacentDiff(const std::vector<double>& in_vec, std::vector<double>& out_vec);
        void reserveMemoryForAllVector();
        void computeConstantA();
        void computeDyDx();
        void computeConstantB();
        void computeConstantC();
        void computeConstantD();
        void outputSplineSet();
        void computePosition(const std::vector<double>& vec);
        void computeFirstDerivative(const std::vector<double>& vec);
        void computeSecondDerivative(const std::vector<double>& vec);
        void computeThirdDerivative();
        int getIndex(const double& t);
    };

    CubicSpline::CubicSpline(const std::vector<double>& x, const std::vector<double>& y,
                             const std::size_t& interpolation_size)
        : m_x(x), m_y(y)
    {
        assert(m_x.size() == m_y.size());
        n_points = m_y.size() - 1;
        reserveMemoryForAllVector();
        computeConstantA();
        adjacentDiff(m_x, m_dx);
        adjacentDiff(m_y, m_dy);
        computeDyDx();
        computeConstantC();
        computeConstantB();
        computeConstantD();
        outputSplineSet();

        m_interval = linspace(m_x.front(), m_x.back(), interpolation_size);
        computeAllConstants();
    }

    void CubicSpline::reserveMemoryForAllVector()
    {
        m_a.reserve(n_points + 1);  // i = 0, ... , n
        m_dx.reserve(n_points);     // i = 0, ..., n - 1
        m_dy.reserve(n_points);     // i = 0, ..., n - 1
        m_c.reserve(n_points + 1);
        m_spline.reserve(n_points);
    }

    void CubicSpline::adjacentDiff(const std::vector<double>& in_vec, std::vector<double>& out_vec)
    {
        std::adjacent_difference(in_vec.cbegin(), in_vec.cend(), std::back_inserter(out_vec));
        out_vec.erase(out_vec.begin());
        assert(out_vec.size() == n_points);
    }

    void CubicSpline::computeConstantA()
    {
        std::copy(m_y.cbegin(), m_y.cend(), std::back_inserter(m_a));
    }

    void CubicSpline::computeDyDx()
    {
        m_dy_dx.reserve(n_points);
        std::transform(m_dy.begin(), m_dy.end(), m_dx.begin(), std::back_inserter(m_dy_dx),
                       [](double a, double b) { return a / b; });
    }

    void CubicSpline::computeConstantB()
    {
        m_b.reserve(n_points + 1);
        for (size_t idx = 0; idx < m_c.size() - 1; ++idx) {
            m_b.push_back(m_dy_dx.at(idx) - m_dx.at(idx) * (2 * m_c.at(idx) + m_c.at(idx + 1)) / 3);
        }
        // https://kluge.in-chemnitz.de/opensource/spline/
        m_b.push_back(0);  // boundary condition at point n;
    }

    void CubicSpline::computeConstantC()
    {
        // Compute matrix A
        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(n_points + 1, n_points + 1);
        A(0, 0) = 1.0;
        std::vector<double> dx_sum;
        dx_sum.reserve(n_points - 1);
        std::transform(m_dx.begin(), m_dx.end() - 1, m_dx.begin() + 1, std::back_inserter(dx_sum),
                       std::plus<double>());

        for (size_t idx_col = 0; idx_col < n_points - 1; ++idx_col) {
            A(idx_col + 1, idx_col) = m_dx.at(idx_col);
            A(idx_col + 1, idx_col + 1) = 2 * dx_sum.at(idx_col);
            A(idx_col + 1, idx_col + 2) = m_dx.at(idx_col + 1);
        }

        A(n_points, n_points) = 1.0;

        // Compute matrix B
        std::vector<double> diff_dy_dx;
        diff_dy_dx.reserve(n_points - 1);

        std::adjacent_difference(m_dy_dx.begin(), m_dy_dx.end(), std::back_inserter(diff_dy_dx));
        diff_dy_dx.erase(diff_dy_dx.begin());
        Eigen::MatrixXf B = Eigen::MatrixXf::Zero(n_points + 1, 1);
        for (size_t idx_row = 0; idx_row < n_points - 1; ++idx_row) {
            B(idx_row + 1, 0) = 3.0f * diff_dy_dx.at(idx_row);
        }

        // Solve AC = B
        Eigen::MatrixXf C = A.colPivHouseholderQr().solve(B);
        C(0, 0) = 0.0f;
        std::vector<double> cc(C.data(), C.data() + C.rows() * C.cols());
        std::copy(cc.cbegin(), cc.cend(), std::back_inserter(m_c));
    }

    void CubicSpline::computeConstantD()
    {
        m_d.reserve(n_points + 1);
        std::vector<double> diff_m_c;
        std::adjacent_difference(m_c.begin(), m_c.end(), std::back_inserter(diff_m_c));
        diff_m_c.erase(diff_m_c.begin());
        std::transform(diff_m_c.begin(), diff_m_c.end(), m_dx.begin(), std::back_inserter(m_d),
                       [](double dc, double x) { return dc / (3 * x); });
        // https://kluge.in-chemnitz.de/opensource/spline/
        m_d.push_back(0);  // boundary condition
    }

    void CubicSpline::outputSplineSet()
    {
        for (size_t idx = 0; idx < n_points + 1; ++idx) {
            m_spline.emplace_back(SplineSet(m_a.at(idx), m_b.at(idx), m_c.at(idx), m_d.at(idx)));
        }
    }

    void CubicSpline::computeAllConstants()
    {
        // x_(i+1)] upper bound or lower bound search where is the occurrence of p
        // in m_x interval
        std::vector<size_t> idx;
        std::transform(m_interval.begin(), m_interval.end(), std::back_inserter(idx),
                       [&](double i) { return getIndex(i); });

        std::vector<double> temp(m_interval.size());

        std::transform(m_interval.begin(), m_interval.end(), idx.begin(), temp.begin(),
                       [&](double interv, size_t i) { return interv - m_x.at(i); });
        std::transform(idx.begin(), idx.end(), std::back_inserter(m_val),
                       [&](size_t i) { return m_spline.at(i); });
        computePosition(temp);
        computeFirstDerivative(temp);
        computeSecondDerivative(temp);
        computeThirdDerivative();
    }

    void CubicSpline::computePosition(const std::vector<double>& vec)
    {
        // S_i(x) = y_i + b_i(x - x_i) + c_i(x- x_i)^2 + d_i(d -d_i)^3 on [x_i,
        m_position.resize(m_interval.size());
        std::transform(m_val.begin(), m_val.end(), vec.begin(), m_position.begin(),
                       [](SplineSet ss, double dx) {
                           return ss.a + (ss.b * dx) + (ss.c * dx * dx) + (ss.d * dx * dx * dx);
                       });
    }

    void CubicSpline::computeFirstDerivative(const std::vector<double>& vec)
    {
        m_first_derivative.resize(m_interval.size());
        std::transform(
            m_val.begin(), m_val.end(), vec.begin(), m_first_derivative.begin(),
            [](SplineSet ss, double dx) { return ss.b + (2 * ss.c * dx) + (3 * ss.d * dx * dx); });
    }
    void CubicSpline::computeSecondDerivative(const std::vector<double>& vec)
    {
        m_second_derivative.resize(m_interval.size());
        std::transform(m_val.begin(), m_val.end(), vec.begin(), m_second_derivative.begin(),
                       [](SplineSet ss, double dx) { return (2 * ss.c) + (6 * ss.d * dx); });
    }

    void CubicSpline::computeThirdDerivative()
    {
        m_third_dev.resize(m_interval.size());
        std::transform(m_val.begin(), m_val.end(), m_third_dev.begin(),
                       [](SplineSet ss) { return 6 * ss.d; });
    }

    int CubicSpline::getIndex(const double& t)
    {
        auto upper = std::upper_bound(m_x.cbegin(), m_x.cend(), t);
        return std::distance(m_x.cbegin(), upper) - 1;
    }
}  // namespace Robotics::Polynomial

#endif /* CUBIC_SPLINE_INTERPOLATION_H */
