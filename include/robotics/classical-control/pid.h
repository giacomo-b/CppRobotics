#pragma once

#include <robotics/common.h>

#include <cmath>
#include <algorithm>

namespace Robotics::ClassicalControl {
    class PID
    {
        public:
            /**
             * @brief Creates a new PID controller
             * @param Kp proportional gain
             * @param Ki integral gain
             * @param Kd derivative gain
             */
            PID(double Kp, double Ki, double Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}

            double ComputeControlAction(double current, double target);

            void SetTimeDiscretization(double step) { dt = step; }

            void SetControlActionLimits(double min, double max) {
                is_limited = true;
                u_min = min;
                u_max = max;
            }
        
        private:
            double Kp, Ki, Kd;
            double dt{0.1};
            bool is_limited{false};
            double integral{0};
            double err_old{0};
            double u_min;
            double u_max;
    };

    double PID::ComputeControlAction(double current, double target)
    {
        double err = target - current;

        double P_term = Kp * err;

        integral += err * dt;
        double I_term = Ki * integral;
        
        double derivative = (err - err_old) / dt;
        double D_term = Kd * derivative;

        double u = P_term + I_term + D_term;

        if (is_limited)
            u = std::clamp(u, u_min, u_max);

        err_old = err;

        return u;
    }
}