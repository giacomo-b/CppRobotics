#include <matplot/matplot.h>
#include <robotics/robotics.h>

int main()
{
    std::vector<double> x{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    std::vector<double> y{0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};

    constexpr std::size_t interpolation_size{101};
    Robotics::Polynomial::CubicSpline2D ccspline(x, y, interpolation_size);

    auto r2x = ccspline.getx_pos();
    auto r2y = ccspline.gety_pos();
    auto interpolated_trajectory_plot = matplot::plot(x, y, "o", r2x, r2y);
    matplot::title("cubic spline plot");
    interpolated_trajectory_plot[1]->line_width(2);
    matplot::xlabel("x");
    matplot::ylabel("y");
    matplot::hold(matplot::on);
    matplot::show();

    matplot::cla();
    auto yaw_plot = matplot::plot(ccspline.getInterval(), ccspline.getyaw());
    matplot::hold(matplot::on);
    matplot::show();

    matplot::cla();
    auto curvature_plot = matplot::plot(ccspline.getInterval(), ccspline.getCurvature());
    matplot::hold(matplot::on);
    matplot::show();
}
