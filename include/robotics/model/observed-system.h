#pragma once

#include <robotics/common.h>
#include <robotics/model/system.h>
#include <random>

namespace Robotics::Model {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int ControlSize, int MeasureSize>
    class ObservedSystem {
        using State = ColumnVector<StateSize>;
        using ExtractionMatrix = Matrix<MeasureSize, StateSize>;
        using Measurement = ColumnVector<MeasureSize>;
    
      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        ObservedSystem(const System<StateSize, ControlSize>& ideal, ExtractionMatrix H, ExtractionMatrix J_H, SquareMatrix<StateSize> Q, SquareMatrix<MeasureSize> R, SquareMatrix<MeasureSize> noise)
            : ideal_system(ideal), H(H), J_H(J_H), Q(Q), R(R), system_noise(noise) {}
        
        const SquareMatrix<StateSize>& GetStateCovariance() const {
            return Q;
        }

        const SquareMatrix<StateSize>& GetObservationCovariance() const {
            return R;
        }

        const ExtractionMatrix& GetExtractionMatrix() const {
            return H;
        }

        const ExtractionMatrix& GetExtractionMatrixJacobian() const {
            return J_H;
        }

        Measurement GetNoisyMeasurement() {
            State x_real = system.GetState();
            return H * x_real + system_noise * random.GetColumnVector<MeasureSize>();
        }
        
    private:
        const SquareMatrix<MeasureSize> system_noise;
        const System<StateSize, ControlSize>& ideal_system;

        // Extraction matrix
        const ExtractionMatrix H;

        // Jacobian of H
        const ExtractionMatrix J_H;

        // State covariance
        const SquareMatrix<StateSize> Q;

        // Observation covariance
        const SquareMatrix<MeasureSize> R;

        Robotics::NormalDistributionRandomGenerator random;
    };
    
}  // namespace Robotics::LinearControl