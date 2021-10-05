#pragma once

#include <robotics/common.h>
#include <robotics/system/nonlinear-system.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace Robotics::Estimation {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int InputSize, int OutputSize>
    class EKF {
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
        State Update(State, Measurement z, Input u)
        {
            // Predicted state estimate
            system.PropagateDynamics(u);
            x_predicted = system.GetState();

            // Predicted covariance estimate
            SquareMatrix<StateSize> J_F = system.GetStateJacobian(u);
            P_predicted = J_F * P_estimate * J_F.transpose() + Q;

            // Update
            z_predicted = system.GetOutputMatrix() * x_predicted;
            residual = z - z_predicted;

            const Matrix<OutputSize, StateSize> J_H = system.GetOutputJacobian(u);
            S = J_H * P_predicted * J_H.transpose() + R;
            K = P_predicted * J_H.transpose() * S.inverse();
            x_estimate = x_predicted + K * residual;
            P_estimate = (SquareMatrix<StateSize>::Identity() - K * J_H) * P_predicted;

            return x_estimate;
        }

        /**
         * @brief Sets the time step used to propagate the dynamics
         * @param step desired time step
         */
        void SetTimeStep(double step) { dt = step; }

      private:
        NonlinearSystem system;

        double dt{0.1};

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