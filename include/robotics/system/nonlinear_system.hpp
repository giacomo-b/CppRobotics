#pragma once

#include <cmath>
#include <functional>
#include <robotics/system/system_base.hpp>

namespace Robotics::Model {

    /**
     * @brief A class for implementing a nonlinear dynamical system
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

        template <typename T>
        using f = std::function<T(const State&, const Input&, double)>;

      public:
        /**
         * @brief Creates a new nonlinear system
         * @param A state matrix as a function of (x, u, dt)
         * @param B control matrix as a function of (x, u, dt)
         * @param C output matrix as a function of (x, u, dt)
         * @param D feedthrough matrix as a function of (x, u, dt)
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
         * @brief Creates a new nonlinear system
         * @param A state matrix as a function of (x, u, dt)
         * @param B control matrix as a function of (x, u, dt)
         * @param C output matrix as a function of (x, u, dt)
         */
        NonlinearSystem(f<StateMatrix> A, f<InputMatrix> B, f<OutputMatrix> C)
            : state_matrix(std::move(A)), input_matrix(std::move(B)), output_matrix(std::move(C))
        {
        }

        /**
         * @brief Creates a new nonlinear system
         * @param A state matrix as a function of (x, u, dt)
         * @param B control matrix as a function of (x, u, dt)
         */
        NonlinearSystem(f<StateMatrix> A, f<InputMatrix> B)
            : state_matrix(std::move(A)), input_matrix(std::move(B))
        {
        }

        /**
         * @brief Sets an expression for the state Jacobian
         * @param J_F state jacobian as a function of (x, u, dt)
         * @todo remove once automatic differentiation is implemented
         * @todo add nullptr check and handling
         */
        void SetStateJacobian(f<SquareMatrix<StateSize>> J_F) { state_jacobian = std::move(J_F); }

        /**
         * @brief Sets an expression for the output Jacobian
         * @param J_H output jacobian as a function of (x, u, dt)
         * @todo remove once automatic differentiation is implemented
         * @todo add nullptr check and handling
         */
        void SetOutputJacobian(f<Matrix<OutputSize, StateSize>> J_H)
        {
            output_jacobian = std::move(J_H);
        }

        /** @copydoc SystemBase::PropagateDynamics(const Input&)
         */
        void PropagateDynamics(const Input& u, double dt)
        {
            this->A = ComputeStateMatrix(u, dt);
            this->B = ComputeInputMatrix(u, dt);
            this->x = this->A * this->x + this->B * u;
        }

        /**
         * @brief Gets the state Jacobian
         * @param u system input
         * @return the state Jacobian
         */
        SquareMatrix<StateSize> GetStateJacobian(const Input& u, double dt) const
        {
            return state_jacobian(this->x, u, dt);
        }

        /**
         * @brief Gets the output Jacobian
         * @param u system input
         * @return the output Jacobian
         */
        OutputMatrix GetOutputJacobian(const Input& u, double dt) const
        {
            return output_jacobian(this->x, u, dt);
        }

      private:
        StateMatrix ComputeStateMatrix(const Input& u, double dt) const
        {
            return state_matrix(this->x, u, dt);
        }

        InputMatrix ComputeInputMatrix(const Input& u, double dt) const
        {
            return input_matrix(this->x, u, dt);
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