#include <robotics/robotics.h>

#include <chrono>
#include <iostream>
#include <random>

int main() {
    static constexpr int N = 2;
    static constexpr int M = 1;

    using ControlAction = Robotics::ColumnVector<M>;

    // Spring-mass-damper parameters
    const double m = 1.0,
                 k = 1.0,
                 b = 0.2;

    // PID parameters
    const double Kp = 35,
                 Ki = 135,
                 Kd = 2;
    
    const double dt = 0.1;

    // State-space formulation
    Robotics::SquareMatrix<N> A;
    A << 0,     0, 
        -k/m, -b/m;
    
    Robotics::Matrix<N, M> B;
    B << 0, 1/m;

    Robotics::ColumnVector<N> x(0.3, 0);
    Robotics::Model::LinearSystem<N, M> system(A, B, x);
    system.SetTimeDiscretization(dt);

    Robotics::ClassicalControl::PID pid(Kp, Ki, Kd);
    pid.SetTimeDiscretization(dt);
    pid.SetControlActionLimits(-100, 100);

    const double target = 0.0;
    const double sim_time = 50;
    double time = 0.0;

    while(time <= sim_time) {
        time += dt;
        double position = system.GetState()(0);
        double u = pid.ComputeControlAction(position, target);
        std::cout << "Current position: " << position << '\t' << "Control action: " << u << '\n';
        system.PropagateDynamics(system.GetState(), ControlAction(u));
    }

    return 0;
}
