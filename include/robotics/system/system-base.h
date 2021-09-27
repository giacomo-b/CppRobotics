#pragma once

#include <robotics/common.h>

#include <Eigen/Dense>

namespace Robotics::Model {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int InputSize, int OutputSize> class SystemBase {
      public:
        using State = ColumnVector<StateSize>;
        using Input = ColumnVector<InputSize>;
        using Output = ColumnVector<OutputSize>;

        using StateMatrix = SquareMatrix<StateSize>;
        using InputMatrix = Matrix<StateSize, InputSize>;
        using OutputMatrix = Matrix<OutputSize, StateSize>;
        using FeedthroughMatrix = Matrix<OutputSize, StateSize>;

        /**
         * @brief Computes the optimal path to reach the target state
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         */
        virtual void PropagateDynamics(const Input& u) = 0;

        void SetTimeDiscretization(double step) { dt = step; };

        void SetInitialState(State state) { x = state; }

        State GetState() const { return x; }

        Output GetOutput()
        {
            y = C * x;
            return y;
        }

        OutputMatrix GetOutputMatrix() const { return C; }

        Output GetNoisyMeasurement(const ColumnVector<OutputSize>& noise)
        {
            return C * x + noise * random.GetColumnVector<OutputSize>();
        }

      protected:
        double dt{0.1};
        State x{State::Zero()};
        Output y{Output::Zero()};

        StateMatrix A;
        InputMatrix B;
        OutputMatrix C;
        FeedthroughMatrix D;

        Robotics::NormalDistributionRandomGenerator random;
    };

}  // namespace Robotics::Model