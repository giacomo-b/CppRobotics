#pragma once

#include <robotics/model/system.h>
#include <functional>
#include <cmath>

namespace Robotics::Model {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int ControlSize>
    class NonlinearSystem : public System<StateSize, ControlSize> {
        using State = System<StateSize, ControlSize>::State;
        using StateMatrix = System<StateSize, ControlSize>::StateMatrix;
        using ControlAction = System<StateSize, ControlSize>::ControlAction;
        using ControlMatrix = System<StateSize, ControlSize>::ControlMatrix;
        using nonlinear_function = std::function<StateMatrix(const State&, const ControlAction&, double)>;
    
      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        NonlinearSystem(nonlinear_function A, nonlinear_function B, nonlinear_function J, State initial)
            : stateMatrixCallback(std::bind(A, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)),
              controlMatrixCallback(std::bind(B, std::ref(std::placeholders::_4), std::ref(std::placeholders::_5), std::placeholders::_6)),
              jacobianCallback(std::bind(J, std::ref(std::placeholders::_7), std::ref(std::placeholders::_8), std::placeholders::_9))
              {
                  this->x = initial;
              }
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        NonlinearSystem(nonlinear_function A, nonlinear_function B, nonlinear_function J)
            : NonlinearSystem(A, B, J, State::Zero()) {}

        /**
         * @brief Computes the optimal path to reach the target state
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         */
        State PropagateDynamics(const State& x0, const ControlAction& u) {
            this->A = ComputeStateMatrix(x0);
            this->B = ComputeControlMatrix(x0);
            this->x = this->A * x0 + this->B * u;
            return this->x;
        }

        SquareMatrix<StateSize> GetJacobian(const ControlAction& u) const { return jacobianCallback(this->x, u, this->dt); }

      private:
        StateMatrix ComputeStateMatrix(const State& x) const {
            return stateMatrixCallback(x);
        }

        ControlMatrix ComputeControlMatrix(const State& x) const {
            return controlMatrixCallback(x, this->dt);
        }
        
        std::function<StateMatrix(const State&, const ControlAction&, double)> stateMatrixCallback;
        std::function<ControlMatrix(const State&, const ControlAction&, double)> controlMatrixCallback;
        std::function<SquareMatrix<StateSize>(const State&, const ControlAction&, double)> jacobianCallback;

        SquareMatrix<StateSize> J; // Jacobian
    };
    
}  // namespace Robotics::LinearControl