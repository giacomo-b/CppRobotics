#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <robotics/common.hpp>
#include <robotics/system/nonlinear_system.hpp>
#include <vector>

namespace Robotics::Estimation {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int InputSize, int OutputSize>
    class EKF {
        static_assert(StateSize > 0);
        static_assert(InputSize > 0);
        static_assert(OutputSize > 0);

        using State = ColumnVector<StateSize>;
        using Input = ColumnVector<InputSize>;
        using Measurement = ColumnVector<OutputSize>;

        using NonlinearSystem = Robotics::Model::NonlinearSystem<StateSize, InputSize, OutputSize>;

      public:
        /**
         * @brief Creates a new Extended Kalman Filter
         * @param system nonlinear model of the system
         * @param Q state covariance matrix
         * @param R output covariance matrix
         */
        EKF(NonlinearSystem system, SquareMatrix<StateSize> Q, SquareMatrix<OutputSize> R)
            : system(system), Q(Q), R(R)
        {
        }

        /**
         * @brief Updates the state estimate
         * @param previous_estimate last estimated state
         * @param z latest measurement
         * @param u control input
         * @return the updated state estimate
         */
        State Update(State, Measurement z, Input u, double dt)
        {
            // Predicted state estimate
            system.PropagateDynamics(u, dt);
            x_predicted = system.GetState();

            // Predicted covariance estimate
            SquareMatrix<StateSize> J_F = system.GetStateJacobian(u, dt);
            P_predicted = J_F * P_estimate * J_F.transpose() + Q;

            // Update
            z_predicted = system.GetOutputMatrix() * x_predicted;
            residual = z - z_predicted;

            const Matrix<OutputSize, StateSize> J_H = system.GetOutputJacobian(u, dt);
            S = J_H * P_predicted * J_H.transpose() + R;
            K = P_predicted * J_H.transpose() * S.inverse();
            x_estimate = x_predicted + K * residual;
            P_estimate = (SquareMatrix<StateSize>::Identity() - K * J_H) * P_predicted;

            return x_estimate;
        }

      private:
        NonlinearSystem system;

        SquareMatrix<StateSize> P_predicted, P_estimate;
        SquareMatrix<OutputSize> S;
        Robotics::Matrix<StateSize, OutputSize> K;

        State x_predicted, x_estimate;
        Measurement z_predicted, residual;

        // State covariance
        const SquareMatrix<StateSize> Q;

        // Observation covariance
        const SquareMatrix<OutputSize> R;
    };

}  // namespace Robotics::Estimation