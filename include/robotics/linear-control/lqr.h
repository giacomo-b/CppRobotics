#pragma once

/*
Original author (Python): Atsushi Sakai (@Atsushi_twi)
Author (C++): Giacomo Bonaccorsi (giacomo-b)
*/

#include <fmt/format.h>
#include <robotics/common.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace Robotics::LinearControl {

    /**
     * @brief A class for path planning using a Linear Quadratric Regulator
     */
    class LQR {
      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        LQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R)
            : A(A), B(B), Q(Q), R(R)
        {
            K = ComputeGain();
        }

        /**
         * @brief Computes the optimal path to reach the target position
         * @param initial the initial coordinates
         * @param target the target coordinates
         * @return a vector containing the coordinates of the whole path
         */
        std::vector<Coordinates> Solve(Coordinates initial, Coordinates target);

        void setTimeLimit(double limit) { max_time = limit; }
        void setTimeStep(double step) { dt = step; }
        void setFinalPositionTolerance(double tol) { goal_dist = tol; }
        void setIterationsLimit(int limit) { max_iter = limit; }
        void setDARETolerance(double tol) { eps = tol; }

      private:
        Eigen::VectorXd ControlAction(Eigen::VectorXd state) const;

        /**
         * @brief Computes the LQR problem gain matrix
         * @return
         */
        Eigen::MatrixXd ComputeGain();

        /**
         * @brief Solves a discrete-time algebraic Riccati equation
         * @return
         */
        Eigen::MatrixXd SolveDARE() const;

        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::MatrixXd K;

        double max_time{100.0};
        double dt{0.1};
        double goal_dist{0.1};
        int max_iter{150};
        double eps{0.01};
    };

    std::vector<Coordinates> LQR::Solve(Coordinates init, Coordinates target)
    {
        std::vector<Coordinates> path{init};
        path.reserve((unsigned int)std::round(max_time / dt) + 1);  // TODO: currently assuming the worst case

        Eigen::VectorXd state(2);
        state << (init.x - target.x), (init.y - target.y);

        bool path_found = false;
        double time = 0;

        Eigen::VectorXd u(2);

        while (time < max_time) {
            time += dt;
            u = ControlAction(state);

            state = A * state + B * u;

            // TODO: assumes the system is 2D, path should be something like vector<vector>
            Coordinates current = {state(0) + target.x, state(1) + target.y};
            path.push_back(current);

            double d = std::sqrt(std::pow((target.x - path.back().x), 2)
                                 + std::pow((target.y - path.back().y), 2));

            if (d <= goal_dist) {
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

    Eigen::VectorXd LQR::ControlAction(Eigen::VectorXd state) const { return -K * state; }

    Eigen::MatrixXd LQR::ComputeGain()
    {
        Eigen::MatrixXd X = SolveDARE();
        return (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
    }

    Eigen::MatrixXd LQR::SolveDARE() const
    {
        Eigen::MatrixXd X = Q, Xn = Q;
        for (auto _ = max_iter; _--;) {
            Xn = A.transpose() * X * A
                 - A.transpose() * X * B * (R + B.transpose() * X * B).inverse() * B.transpose() * X
                       * A
                 + Q;
            if ((Xn - X).lpNorm<Eigen::Infinity>() < eps) break;
            X = Xn;
        }
        return Xn;
    }

}  // namespace Robotics::LinearControl