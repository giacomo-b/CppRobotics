#pragma once

#include <Eigen/Dense>
#include <robotics/common.h>

namespace Robotics::Model {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int ControlSize>
    class System {
    
      public:
        using State = ColumnVector<StateSize>;
        using StateMatrix = SquareMatrix<StateSize>;
        using ControlAction = ColumnVector<ControlSize>;
        using ControlMatrix = Matrix<StateSize, ControlSize>;
        /**
         * @brief Computes the optimal path to reach the target state
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         */
        virtual State PropagateDynamics(const State& x0, const ControlAction& u) = 0;

        void SetTimeDiscretization(double step) { dt = step; };
        void SetState(State state) { x = state; }
        State GetState() const {  return x; };
        
      protected:
        double dt{0.1};
        State x{State::Zero()};
        StateMatrix A;
        ControlMatrix B;
    };
    
}  // namespace Robotics::LinearControl