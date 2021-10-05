#include <matplot/matplot.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <robotics/robotics.hpp>

#include "dynamics.hpp"

Input sample_input();

int main()
{
    using Robotics::ColumnVector;
    using Robotics::SquareMatrix;

    const double dt = 0.1;
    const double sim_time = 50;
    const unsigned int n_steps = (unsigned int)std::round(sim_time / dt) + 1;

    // Nonlinear system with specified initial state (x pos, y pos, yaw, velocity)
    State x_real = ColumnVector<N>::Zero();
    Robotics::Model::NonlinearSystem<N, M, P> system(A, B, C);
    system.SetStateJacobian(state_jacobian);
    system.SetOutputJacobian(output_jacobian);

    Robotics::Model::NonlinearSystem<N, M, P> dr_system = system;

    // State and output covariance
    const SquareMatrix<N> Q = ColumnVector<N>(0.1, 0.1, deg2rad(1.0), 1.0).cwiseAbs2().asDiagonal();
    const SquareMatrix<P> R = ColumnVector<P>(1.0, 1.0).cwiseAbs2().asDiagonal();

    const SquareMatrix<M> input_disturbance
        = ColumnVector<M>(1.0, deg2rad(30)).cwiseAbs2().asDiagonal();
    const SquareMatrix<P> gps_noise = ColumnVector<P>(0.5, 0.5).cwiseAbs2().asDiagonal();

    // Estimated and dead-reckoning states
    State x_estimated = ColumnVector<N>::Zero();
    State x_dr = ColumnVector<N>::Zero();

    // Measurement
    ColumnVector<P> z = ColumnVector<P>::Zero();

    // For logging
    std::vector<State> ekf_history, real_history, dr_history;
    std::vector<Output> measurements;

    ekf_history.reserve(n_steps);
    ekf_history.push_back(x_estimated);

    real_history.reserve(n_steps);
    real_history.push_back(x_real);

    dr_history.reserve(n_steps);
    dr_history.push_back(x_real);

    measurements.reserve(n_steps);
    measurements.push_back(z);

    Robotics::Estimation::EKF<N, M, P> filter(system, Q, R);

    Input u, u_noisy;
    Robotics::NormalDistributionRandomGenerator rand;

    double time = 0.0;

    while (time <= sim_time) {
        time += dt;

        // Select new control action
        u = sample_input();

        // Compute the ideal new state
        system.PropagateDynamics(u, dt);
        x_real = system.GetState();

        // Observe the ideal state through the measurement system
        z = system.GetNoisyMeasurement(gps_noise);

        // Add disturbance on top of the ideal input
        u_noisy = u + input_disturbance * rand.GetColumnVector<M>();

        // Compute the new dead-reckoning state
        dr_system.PropagateDynamics(u_noisy, dt);
        x_dr = dr_system.GetState();

        // New estimate
        x_estimated = filter.Update(x_estimated, z, u_noisy, dt);

        ekf_history.push_back(x_estimated);
        real_history.push_back(x_real);
        dr_history.push_back(x_dr);
        measurements.push_back(z);
    }

    return 0;
}

Input sample_input()
{
    double v = 1.0;        // [m/s]
    double yawrate = 0.1;  // [rad/s]

    return Input(v, yawrate);
}