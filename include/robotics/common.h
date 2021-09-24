#pragma once

#include <Eigen/Dense>
#include <iomanip>
#include <vector>

namespace Robotics {

    struct Coordinates {
        double x;
        double y;
    };

}  // namespace Robotics

std::ostream& operator<<(std::ostream& os, const std::vector<Robotics::Coordinates>& coords)
{
    for (const auto& coord : coords) {
        os << std::right << std::setw(10) << std::fixed << std::setprecision(4) << coord.x
           << std::right << std::setw(10) << std::fixed << std::setprecision(4) << coord.y << '\n';
    }
    return os;
}