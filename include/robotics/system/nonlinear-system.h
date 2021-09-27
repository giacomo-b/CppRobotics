#pragma once

#include <robotics/system/system-base.h>

#include <cmath>
#include <functional>

namespace Robotics::Model {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int InputSize, int OutputSize>
    class NonlinearSystem : public SystemBase<StateSize, InputSize, OutputSize> {
        using System = SystemBase<StateSize, InputSize, OutputSize>;

        using State = typename System::State;
        using Input = typename System::Input;
        using Output = typename System::Output;

        using StateMatrix = typename System::StateMatrix;
        using InputMatrix = typename System::InputMatrix;
        using OutputMatrix = typename System::OutputMatrix;
        using FeedthroughMatrix = typename System::FeedthroughMatrix;

        template <typename T> using f = std::function<T(const State&, const Input&, double)>;

      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        NonlinearSystem(f<StateMatrix> A, f<InputMatrix> B, f<OutputMatrix> C,
                        f<FeedthroughMatrix> D)
            : state_matrix(std::move(A)),
              input_matrix(std::move(B)),
              output_matrix(std::move(C)),
              feedthrough_matrix(std::move(D))
        {
        }

        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        NonlinearSystem(f<StateMatrix> A, f<InputMatrix> B, f<OutputMatrix> C)
            : state_matrix(std::move(A)),
              input_matrix(std::move(B)),
              output_matrix(std::move(C))
        {
        }

        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        NonlinearSystem(f<StateMatrix> A, f<InputMatrix> B)
            : state_matrix(std::move(A)),
              input_matrix(std::move(B))
        {
        }

        // TODO: to be removed once automatic differentiation is implemented
        void SetStateJacobian(f<SquareMatrix<StateSize>> J_F)
        {
            state_jacobian = std::move(J_F);
        }

        // TODO: to be removed once automatic differentiation is implemented
        void SetOutputJacobian(f<Matrix<OutputSize, StateSize>> J_H)
        {
            output_jacobian = std::move(J_H);
        }

        /**
         * @brief Computes the optimal path to reach the target state
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         */
        void PropagateDynamics(const Input& u)
        {
            this->A = ComputeStateMatrix(u);
            this->B = ComputeInputMatrix(u);
            this->x = this->A * this->x + this->B * u;
        }

        SquareMatrix<StateSize> GetStateJacobian(const Input& u) const
        {
            return state_jacobian(this->x, u, this->dt);
        }

        OutputMatrix GetOutputJacobian(const Input& u) const {
            return output_jacobian(this->x, u, this->dt);
        }

      private:
        StateMatrix ComputeStateMatrix(const Input& u) const
        {
            return state_matrix(this->x, u, this->dt);
        }

        InputMatrix ComputeInputMatrix(const Input& u) const
        {
            return input_matrix(this->x, u, this->dt);
        }

        f<StateMatrix> state_matrix;
        f<InputMatrix> input_matrix;
        f<OutputMatrix> output_matrix;
        f<FeedthroughMatrix> feedthrough_matrix;

        f<SquareMatrix<StateSize>> state_jacobian;
        f<Matrix<OutputSize, StateSize>> output_jacobian;

        SquareMatrix<StateSize> J_F;  // Jacobian
        OutputMatrix J_H;
    };

}  // namespace Robotics::Model