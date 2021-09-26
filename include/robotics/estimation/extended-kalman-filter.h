#pragma once

#include <robotics/common.h>
#include <robotics/model/nonlinear-system.h>
#include <robotics/model/observed-system.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace Robotics::Estimation {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int ControlSize, int MeasureSize>
    class EKF {
        using State = ColumnVector<StateSize>;
        using ControlAction = ColumnVector<ControlSize>;
        using Measurement = ColumnVector<MeasureSize>;
    
      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        EKF(NonlinearSystem system, ObservedSystem observed)
            : system(system), observed_system(observed) {}
        
        State Update(State previous_estimate, Measurement z, ControlAction u) {

            // Predicted state estimate
            x_predicted = system.PropagateDynamics(previous_estimate, u);

            // Predicted covariance estimate
            SquareMatrix<StateSize> J_F = system.GetJacobian(u);
            P_predicted = J_F * P_estimate * J_F.transpose() + observed_system.GetStateCovariance();

            // Update
            z_predicted = observed_system.GetExractionMatrix() * x_predicted;
            residual = z - z_predicted;

            const Matrix<MeasureSize, StateSize>& J_H = observed_system.GetExtractionMatrixJacobian();
            S = J_H * P_predicted * J_H.transpose() + observed_system.GetObservationCovariance();
            K = P_predicted * J_H.transpose() * S.inverse();
            x_estimate = x_predicted + K * residual;
            P_estimate = (ColumnVector<State>::Identity() - K * J_H) * P_predicted;

            return x_estimate;
        }

        void SetTimeStep(double step) { dt = step; }

      private:
        NonlinearSystem system;
        ObservedSystem observed_system;

        double dt{0.1};

        SquareMatrix<StateSize> P_predicted, P_estimate;
        SquareMatrix<MeasureSize> S;
        Robotics::Matrix<StateSize, MeasureSize> K;


        State x_predicted, x_estimate;
        Measurement z_predicted, residual;
    };
    
}  // namespace Robotics::LinearControl