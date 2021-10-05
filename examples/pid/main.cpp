#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <robotics/robotics.hpp>

int main()
{
    static constexpr int N = 1;
    static constexpr int M = 1;
    static constexpr int P = 1;

    using Input = Robotics::ColumnVector<M>;

    const double Kp = 0.2, Ki = 0.4, Kd = 0.02;
    const double dt = 0.1;

    // State-space formulation
    Robotics::SquareMatrix<N> A;
    A << 1;

    Robotics::Matrix<N, M> B;
    B << 1;

    Robotics::Matrix<P, N> C;
    C << 1;

    Robotics::Model::LinearSystem<N, M, P> system(A, B, C);
    system.SetInitialState(Robotics::ColumnVector<N>(0.0));

    Robotics::ClassicalControl::PID pid(Kp, Ki, Kd);
    pid.SetControlActionLimits(-50.0, 50);

    const double target = 10.0;
    const double sim_time = 1;
    double time = 0.0;

    while (time <= sim_time) {
        time += dt;
        double position = system.GetOutput()(0);
        Input u = Input(pid.ComputeControlAction(position, target, dt));
        std::cout << "Current position: " << std::fixed << std::setprecision(20) << position << '\t'
                  << "Control action: " << u << '\n';
        system.PropagateDynamics(u, dt);
    }

    return 0;
}
