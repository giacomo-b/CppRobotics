#pragma once

#include <robotics/system/system-base.h>
#include <functional>
#include <cmath>

namespace Robotics::Model {

    /**
     * @brief A class for implemeting an Extended Kalman Filter
     */
    template <int StateSize, int InputSize, int OutputSize>
    class NonlinearSystem : public SystemBase<StateSize, InputSize, OutputSize> {
        
        using SystemBase = SystemBase<StateSize, InputSize, OutputSize>;

        using State = SystemBase::State;
        using Input = SystemBase::Input;
        using Output = SystemBase::Output;

        using StateMatrix = SystemBase::StateMatrix;
        using InputMatrix = SystemBase::InputMatrix;
        using OutputMatrix = SystemBase::OutputMatrix;
        using FeedthroughMatrix = SystemBase::FeedthroughMatrix;
        
        template<typename T>
        using f = std::function<T(const State&, const Input&, double)>;
    
      public:
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        NonlinearSystem(f<StateMatrix> A, f<InputMatrix> B, f<OutputMatrix> C, f<FeedthroughMatrix> D)
            : state_matrix(std::bind(A, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)),
              input_matrix(std::bind(B, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)),
              output_matrix(std::bind(C, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)),
              feedthrough_matrix(std::bind(D, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3))
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
            : state_matrix(std::bind(A, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)),
              input_matrix(std::bind(B, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)),
              output_matrix(std::bind(C, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)) {}
        
        /**
         * @brief Creates a new LQR path planner
         * @param A state matrix
         * @param B control matrix
         * @param Q state weights matrix
         * @param R control weights matrix
         */
        NonlinearSystem(f<StateMatrix> A, f<InputMatrix> B)
            : state_matrix(std::bind(A, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)),
              input_matrix(std::bind(B, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3)) {}
        
        // TODO: to be removed once automatic differentiation is implemented
        void SetStateJacobian(f<SquareMatrix<StateSize>> J_F) {
            state_jacobian = std::bind(J_F, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3);
        }

        // TODO: to be removed once automatic differentiation is implemented
        void SetOutputJacobian(f<Matrix<OutputSize, StateSize>> J_H) {
            output_jacobian = std::bind(J_H, std::ref(std::placeholders::_1), std::ref(std::placeholders::_2), std::placeholders::_3);
        }

        /**
         * @brief Computes the optimal path to reach the target state
         * @param initial the initial state
         * @param target the target state
         * @return a vector containing the state along the whole path
         */
        State PropagateDynamics(const State& x0, const Input& u) {
            this->A = ComputeStateMatrix(x0);
            this->B = ComputeInputMatrix(x0);
            this->x = this->A * x0 + this->B * u;
            return this->x;
        }

        SquareMatrix<StateSize> GetJacobian(const Input& u) const { return jacobianCallback(this->x, u, this->dt); }

        const OutputMatrix& GetOutputMatrixJacobian() const {
            return J_H;
        }

      private:
        StateMatrix ComputeStateMatrix(const State& x) const {
            return stateMatrixCallback(x);
        }

        InputMatrix ComputeInputMatrix(const State& x) const {
            return InputMatrixCallback(x, this->dt);
        }
        
        f<StateMatrix> state_matrix;
        f<InputMatrix> input_matrix;
        f<OutputMatrix> output_matrix;
        f<FeedthroughMatrix> feedthrough_matrix;

        f<SquareMatrix<StateSize>> state_jacobian;
        f<Matrix<OutputSize, StateSize>> output_jacobian;

        SquareMatrix<StateSize> J_F; // Jacobian
        OutputMatrix J_H;
    };
    
}  // namespace Robotics::LinearControl