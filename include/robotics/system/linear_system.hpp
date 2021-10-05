#pragma once

#include <robotics/system/system_base.hpp>

namespace Robotics::Model {

    /**
     * @brief A class for implementing a linear dynamical system
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
         * @brief Creates a new linear system
         * @param A state matrix
         * @param B control matrix
         * @param C output matrix
         * @param D feedthrough matrix
         */
        LinearSystem(StateMatrix A, InputMatrix B, OutputMatrix C, FeedthroughMatrix D)
        {
            this->A = A;
            this->B = B;
            this->C = C;
            this->D = D;
        }

        /**
         * @brief Creates a new linear system
         * @param A state matrix
         * @param B control matrix
         */
        LinearSystem(StateMatrix A, InputMatrix B, OutputMatrix C)
            : LinearSystem(A, B, C, FeedthroughMatrix::Zero())
        {
        }

        /**
         * @brief Creates a new linear system
         * @param A state matrix
         * @param B control matrix
         */
        LinearSystem(StateMatrix A, InputMatrix B)
            : LinearSystem(A, B, OutputMatrix::Zero(), FeedthroughMatrix::Zero())
        {
        }

        /** @copydoc SystemBase::PropagateDynamics(const Input&, double dt)
         */
        void PropagateDynamics(const Input& u, double)
        {
            this->x = this->A * this->x + this->B * u;
        }
    };

}  // namespace Robotics::Model