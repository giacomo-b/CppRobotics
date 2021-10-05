#pragma once

static constexpr int N = 4;  // State size
static constexpr int M = 2;  // Control action size
static constexpr int P = 2;  // Measurement size

using State = Robotics::ColumnVector<N>;
using Input = Robotics::ColumnVector<M>;
using Output = Robotics::ColumnVector<P>;

using StateMatrix = Robotics::SquareMatrix<N>;
using InputMatrix = Robotics::Matrix<N, M>;
using OutputMatrix = Robotics::Matrix<P, N>;
using FeedthroughMatrix = Robotics::Matrix<P, N>;

using Robotics::deg2rad;

StateMatrix A(const State& state, const Input& control, double dt);
InputMatrix B(State state, const Input& control, double dt);
OutputMatrix C(State state, const Input& control, double dt);

StateMatrix state_jacobian(const State& state, const Input& control, double dt);
OutputMatrix output_jacobian(const State& state, const Input& control, double dt);

StateMatrix A(const State&, const Input&, double)
{
    StateMatrix A = Robotics::ColumnVector<N>(1.0, 1.0, 1.0, 0.0).asDiagonal();
    return A;
}

InputMatrix B(State state, const Input&, double dt)
{
    InputMatrix B;

    // clang-format off
    B << dt * cos(state(2)),    0.0,
         dt * sin(state(2)),    0.0,
         0.0,                   dt,
         1.0,                   0.0;
    // clang-format on

    return B;
}

OutputMatrix C(State, const Input&, double)
{
    OutputMatrix H;
    // clang-format off
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    // clang-format on
    return H;
}

StateMatrix state_jacobian(const State& state, const Input& control, double dt)
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

    Robotics::SquareMatrix<N> JF;

    double yaw = state(2);
    double v = control(0);

    // clang-format off
    JF << 1.0, 0.0, -dt * v * sin(yaw), dt * cos(yaw),
          0.0, 1.0,  dt * v * cos(yaw), dt * sin(yaw),
          0.0, 0.0,  1.0,               0.0,
          0.0, 0.0,  0.0,               1.0;
    // clang-format on

    return JF;
}

OutputMatrix output_jacobian(const State&, const Input&, double)
{
    OutputMatrix JH;
    // clang-format off
    JH << 1, 0, 0, 0,
          0, 1, 0, 0;
    // clang-format on
    return JH;
}