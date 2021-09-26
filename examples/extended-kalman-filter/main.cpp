#include <matplot/matplot.h>
#include <robotics/robotics.h>

#include <chrono>
#include <iostream>
#include <iomanip>

static constexpr int N = 4; // State size
static constexpr int M = 2; // Control action size
static constexpr int P = 2; // Measurement size

// Auxiliary aliases
using State = Robotics::ColumnVector<N>;
using StateMatrix = Robotics::SquareMatrix<N>;
using ControlAction = Robotics::ColumnVector<M>;
using ControlMatrix = Robotics::Matrix<N, M>;

Robotics::ColumnVector<M> sample_input();
Robotics::Matrix<N, N> state_matrix(const Robotics::ColumnVector<N>& state, const Robotics::ColumnVector<M>& control, double dt);
Robotics::Matrix<N, N> state_matrix_jacobian(const Robotics::ColumnVector<N>& state, const Robotics::ColumnVector<M>& control, double dt);
Robotics::Matrix<N, M> control_matrix(Robotics::ColumnVector<N> state, const Robotics::ColumnVector<M>& control, double dt);

using Robotics::deg2rad;

int main()
{
    using Robotics::SquareMatrix;
    using Robotics::ColumnVector;

    const double dt = 0.1;
    const double sim_time = 50;
    const unsigned int n_steps = (unsigned int)std::round(sim_time / dt) + 1;

    // Nonlinear system with specified initial state (x pos, y pos, yaw, velocity)
    State x_real = ColumnVector<N>::Zero();
    Robotics::Model::NonlinearSystem<N, M> system(state_matrix, control_matrix, x_real);
    system.SetTimeDiscretization(dt);

    // State and observation covariance 
    const SquareMatrix<N> Q = ColumnVector<N>(0.1, 0.1, deg2rad(1.0), 1.0).pow(2).asDiagonal();
    const SquareMatrix<P> R = ColumnVector<P>(1.0, 1.0).pow(2).asDiagonal();

    const SquareMatrix<M> input_disturbance = ColumnVector<M>(1.0, deg2rad(30)).pow(2).asDiagonal();
    const SquareMatrix<P> gps_noise = ColumnVector<P>(0.5, 0.5).pow(2).asDiagonal();

    // Observed states extraction matrix
    Robotics::Matrix<P, N> H;
    // clang-format off
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    // clang-format on

    Robotics::Matrix<P, N> J_H = H; // Could be different
    
    // Observed system, with noise added on top of the ideal system
    Robotics::Model::ObservedSystem<N, M, P> observed_system(system, H, J_H, Q, R, gps_noise);

    // Estimated and dead-reckoning states
    State x_estimated = ColumnVector<N>::Zero();
    State x_dr = ColumnVector<N>::Zero();
    
    // Measurement
    ColumnVector<P> z = ColumnVector<P>::Zero();

    // For logging
    std::vector<State> ekf_history, real_history, dr_history, measurements;

    ekf_history.reserve(n_steps);
    ekf_history.push_back(x_estimated);

    real_history.reserve(n_steps);
    real_history.push_back(x_real);

    dr_history.reserve(n_steps);
    dr_history.push_back(x_real);

    measurements.reserve(n_steps);
    measurements.push_back(z);

    Robotics::Estimation::EKF<N, M, P> filter(system, observed_system);
    filter.SetTimeStep(dt);

    ControlAction u, u_noisy;
    Robotics::NormalDistributionRandomGenerator rand;

    double time = 0.0;

    while(time <= sim_time) {
        time += dt;

        // Select new control action
        u = sample_input();

        // Compute the ideal new state
        x_real = system.PropagateDynamics(x_real, u);

        // Observe the ideal state through the measurement system
        z = observed_system.GetNoisyMeasurement();

        // Add disturbance on top of the ideal input
        u_noisy = u + input_disturbance * rand.GetColumnVector<M>();

        // Compute the new dead-reckoning state
        x_dr = system.PropagateDynamics(x_dr, u_noisy);

        // New estimate
        x_estimated = filter.Update(x_estimated, z, u_noisy);

        ekf_history.push_back(x_estimated);
        real_history.push_back(x_real);
        dr_history.push_back(x_dr);
        measurements.push_back(z);        
    }

    return 0;
}

Robotics::ColumnVector<M> sample_input() {
    double v = 1.0;         // [m/s]
    double yawrate = 0.1;   // [rad/s]

    Robotics::ColumnVector<M> u;
    u << v, yawrate;

    return u;
}

Robotics::Matrix<N, N> state_matrix(const Robotics::ColumnVector<N>& state, const Robotics::ColumnVector<M>& control, double dt)
{
    Robotics::Matrix<N, N> A = Robotics::ColumnVector<N>(1.0, 1.0, 1.0, 0.0).asDiagonal();
    return A;
}

Robotics::Matrix<N, M> control_matrix(Robotics::ColumnVector<N> state, const Robotics::ColumnVector<M>& control, double dt) {
    Robotics::Matrix<N, M> B;

    // clang-format off
    B << dt * cos(state(2)),    0.0,
         dt * sin(state(2)),    0.0,
         0.0,                   dt,
         1.0,                   0.0;
    // clang-format on

    return B;
}

Robotics::Matrix<N, N> state_matrix_jacobian(const Robotics::ColumnVector<N>& state, const Robotics::ColumnVector<M>& control, double dt)
{
    /*
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    */

    Robotics::SquareMatrix<N> Jacobian;

    double yaw = state(2);
    double v = control(0);
    
    // clang-format off
    Jacobian << 1.0, 0.0, -dt * v * sin(yaw), dt * cos(yaw),
                0.0, 1.0,  dt * v * cos(yaw), dt * sin(yaw),
                0.0, 0.0,  1.0,               0.0,
                0.0, 0.0,  0.0,               1.0;
    // clang-format on

    return Jacobian;
}