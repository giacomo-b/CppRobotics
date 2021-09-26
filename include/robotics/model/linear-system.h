#pragma once

#include <robotics/model/system.h>

namespace Robotics::Model {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int ControlSize>
    class LinearSystem : public System<StateSize, ControlSize> {
        using State = System<StateSize, ControlSize>::State;
        using StateMatrix = System<StateSize, ControlSize>::StateMatrix;
        using ControlAction = System<StateSize, ControlSize>::ControlAction;
        using ControlMatrix = System<StateSize, ControlSize>::ControlMatrix;
    
      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        LinearSystem(StateMatrix A, ControlMatrix B, State initial) {
                this->A = A;
                this->B = B;
                this->x = initial;
        }

        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        LinearSystem(StateMatrix A, ControlMatrix B)
            : LinearSystem(A, B, State::Zero()) {}

        /**
         * @brief Computes the optimal path to reach the target state
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         */
        State PropagateDynamics(const State& x0, const ControlAction& u) {
            this->x = this->A * x0 + this->B * u;
            return this->x;
        }
    };
    
}  // namespace Robotics::LinearControl