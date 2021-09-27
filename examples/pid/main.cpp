#include <robotics/robotics.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

int main()
{
    static constexpr int N = 2;
    static constexpr int M = 1;
    static constexpr int P = 1;

    using Input = Robotics::ColumnVector<M>;

    // Spring-mass-damper parameters
    const double m = 1.0, k = 20.0, b = 10.0;

    // PID parameters
    const double Kp = 300, Ki = 1000, Kd = 0.1;

    const double dt = 0.1;

    // State-space formulation
    Robotics::SquareMatrix<N> A;
    A << 0, 0, -k / m, -b / m;

    Robotics::Matrix<N, M> B;
    B << 0, 1 / m;

    Robotics::Matrix<P, N> C;
    C << 1, 0;

    Robotics::Model::LinearSystem<N, M, P> system(A, B, C);
    system.SetInitialState(Robotics::ColumnVector<N>(0.0, 0.0));
    system.SetTimeDiscretization(dt);

    Robotics::ClassicalControl::PID pid(Kp, Ki, Kd);
    pid.SetTimeDiscretization(dt);
    pid.SetControlActionLimits(-100, 100);

    const double target = 1.0;
    const double sim_time = 1;
    double time = 0.0;

    while (time <= sim_time) {
        time += dt;
        double position = system.GetOutput()(0);
        double u = pid.ComputeControlAction(position, target);
        std::cout << "Current position: " << std::fixed << std::setprecision(20) << position << '\t'
                  << "Control action: " << u << '\n';
        system.PropagateDynamics(system.GetState(), Input(u));
    }

    return 0;
}
