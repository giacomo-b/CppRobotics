#pragma once

#include <robotics/common.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>

namespace Robotics::LinearControl {

    /**
     * @brief A class for path planning using a Linear Quadratic Regulator
     */
    template <int N, int M>
    class LQR {
        using VectorNx1 = ColumnVector<N>;
        using VectorMx1 = ColumnVector<M>;

        using MatrixNxN = SquareMatrix<N>;
        using MatrixMxM = SquareMatrix<M>;
        using ControlMatrix = Matrix<N,M>;
        using GainMatrix = Matrix<M,N>;

        using State = VectorNx1;
      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        LQR(MatrixNxN A, ControlMatrix B, MatrixNxN Q, MatrixMxM R)
            : A(A), B(B), Q(Q), R(R), K(ComputeGain()) {}

        /**
         * @brief Computes the optimal path to reach the target state
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         */
        std::vector<State> Solve(State initial, State target)
        {
            std::vector<State> path{initial};
            path.reserve((unsigned int)std::round(max_time / dt) + 1);  // TODO: currently assuming the worst case

            State x = initial - target;
            VectorMx1 u;

            bool path_found = false;
            double time = 0;
            double goal_dist_squared = std::pow(goal_dist, 2);

            while (time < max_time) {
                time += dt;

                u = ControlAction(x);
                x = A * x + B * u;

                path.push_back(x + target);

                if (x.squaredNorm() <= goal_dist_squared) {
                    path_found = true;
                    break;
                }
            }

            if (!path_found) {
                std::cerr << "Couldn't find a path\n";
                return {};
            }

            return path;
        }

        void SetTimeLimit(double limit) { max_time = limit; }
        void SetTimeStep(double step) { dt = step; }
        void SetFinalPositionTolerance(double tol) { goal_dist = tol; }
        void SetIterationsLimit(int limit) { max_iter = limit; }
        void SetDARETolerance(double tol) { eps = tol; }

      private:
        VectorMx1 ControlAction(State x) const { return -K * x; }

        /**
         * @brief Computes the LQR problem gain matrix
         * @return
         */
        GainMatrix ComputeGain()
        {
            MatrixNxN X = SolveDARE();
            return (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
        }

        /**
         * @brief Solves a discrete-time algebraic Riccati equation
         * @return
         */
        MatrixNxN SolveDARE() const
        {
            MatrixNxN X = Q, Xn = Q;
            for (auto _ = 0; _ < max_iter; _++) {
                std::cout << _ << std::endl;
                Xn = A.transpose() * X * A - A.transpose() * X * B * (R + B.transpose() * X * B).inverse() * B.transpose() * X * A + Q;
                if ((Xn - X).lpNorm<Eigen::Infinity>() < eps) break;
                X = Xn;
            }
            return Xn;
        }

        MatrixNxN A;
        ControlMatrix B;
        MatrixNxN Q;
        MatrixMxM R;
        GainMatrix K;

        double max_time{100.0};
        double dt{0.1};
        double goal_dist{0.1};
        int max_iter{10};
        double eps{0.01};
    };
    
}  // namespace Robotics::LinearControl