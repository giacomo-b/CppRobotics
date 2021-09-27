#pragma once

#include <robotics/system/system-base.h>

namespace Robotics::Model {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int InputSize, int OutputSize>
    class LinearSystem : public SystemBase<StateSize, InputSize, OutputSize> {
        using System = SystemBase<StateSize, InputSize, OutputSize>;

        using State = typename System::State;
        using Input = typename System::Input;
        using Output = typename System::Output;

        using StateMatrix = typename System::StateMatrix;
        using InputMatrix = typename System::InputMatrix;
        using OutputMatrix = typename System::OutputMatrix;
        using FeedthroughMatrix = typename System::FeedthroughMatrix;

      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        LinearSystem(StateMatrix A, InputMatrix B, OutputMatrix C, FeedthroughMatrix D)
        {
            this->A = A;
            this->B = B;
            this->C = C;
            this->D = D;
        }

        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        LinearSystem(StateMatrix A, InputMatrix B, OutputMatrix C)
            : LinearSystem(A, B, C, FeedthroughMatrix::Zero())
        {
        }

        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        LinearSystem(StateMatrix A, InputMatrix B)
            : LinearSystem(A, B, OutputMatrix::Zero(), FeedthroughMatrix::Zero())
        {
        }

        /**
         * @brief Computes the optimal path to reach the target state
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         */
        void PropagateDynamics(const Input& u) { this->x = this->A * this->x + this->B * u; }
    };

}  // namespace Robotics::Model