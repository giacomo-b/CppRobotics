#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <robotics/common.hpp>
#include <vector>

namespace Robotics::LinearControl {

    /**
     * @brief A class for implementing a Linear-Quadratic Regulator
     * @todo: Refactor class to conform to the other controllers
     */
    template <int StateSize, int InputSize>
    class LQR {
        static_assert(StateSize > 0);
        static_assert(InputSize > 0);

        using VectorNx1 = ColumnVector<StateSize>;
        using VectorMx1 = ColumnVector<InputSize>;

        using MatrixNxN = SquareMatrix<StateSize>;
        using MatrixMxM = SquareMatrix<InputSize>;
        using InputMatrix = Matrix<StateSize, InputSize>;
        using GainMatrix = Matrix<InputSize, StateSize>;

        using State = VectorNx1;

      public:
        /**
         * @brief Creates a new Linear-Quadratic Regulator
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         * @todo pass a linear system as input
         */
        LQR(MatrixNxN A, InputMatrix B, MatrixNxN Q, MatrixMxM R)
            : A(A), B(B), Q(Q), R(R), K(ComputeGain())
        {
        }

        /**
         * @brief Solves the LQR problem
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         * @todo this behavior should be in a user-defined loop, refactor so that only one step is
         * carried out at a time
         */
        std::vector<State> Solve(State initial, State target, double dt)
        {
            std::vector<State> path{initial};
            path.reserve((unsigned int)std::round(max_time / dt)
                         + 1);  // TODO: currently assuming the worst case

            State x = initial - target;
            VectorMx1 u;

            bool path_found = false;
            double time = 0;
            double goal_dist_squared = std::pow(goal_dist, 2);

            while (time < max_time) {
                time += dt;

                u = Input(x);
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

        /**
         * @brief Sets the maximum simulation time
         * @param limit time limit
         * @todo remove
         */
        void SetTimeLimit(double limit) { max_time = limit; }

        /**
         * @brief Sets the tolerance w.r.t. the target
         * @param tol tolerance
         * @todo remove, the user should decide when to exit
         */
        void SetFinalPositionTolerance(double tol) { goal_dist = tol; }

        /**
         * @brief Sets the maximum number of iterations
         * @param limit maximum number of iterations
         * @todo remove, the user should decide when to exit
         */
        void SetIterationsLimit(int limit) { max_iter = limit; }

        /**
         * @brief Sets the tolerance for the resolution of the Discrete Algebraic Riccati Equation
         * @param tol tolerance
         */
        void SetDARETolerance(double tol) { eps = tol; }

      private:
        VectorMx1 Input(State x) const { return -K * x; }

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
                Xn = A.transpose() * X * A
                     - A.transpose() * X * B * (R + B.transpose() * X * B).inverse() * B.transpose()
                           * X * A
                     + Q;
                if ((Xn - X).template lpNorm<Eigen::Infinity>() < eps) break;
                X = Xn;
            }
            return Xn;
        }

        MatrixNxN A;
        InputMatrix B;
        MatrixNxN Q;
        MatrixMxM R;
        GainMatrix K;

        double max_time{100.0};
        double goal_dist{0.1};
        int max_iter{10};
        double eps{0.01};
    };

}  // namespace Robotics::LinearControl