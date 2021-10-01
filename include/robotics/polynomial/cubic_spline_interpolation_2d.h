#ifndef CUBIC_SPLINE_INTERPOLATION_2D_H
#define CUBIC_SPLINE_INTERPOLATION_2D_H

#include "cubic_spline_interpolation.h"

namespace Robotics::Polynomial {
class CubicSpline2D {
 public:
  CubicSpline2D(const std::vector<double>& x,
                const std::vector<double>& y,
                const std::size_t& interpolation_size);

  /**
   * @brief compute the euclidean distance between two points and get partial sum.
   */
  void s(const std::vector<double>& x,
         const std::vector<double>& y);

  void computeCurvature();

  std::vector<double> getx_pos() {return x_pos;}
  std::vector<double> gety_pos() {return y_pos;}
  std::vector<double> getyaw() { return yaw; }
  std::vector<double> getInterval() { return m_s_interval; }
  std::vector<double> getCurvature() { return curvature; }

private:
  std::vector<double> m_s {0.0};
  std::vector<double> m_s_interval {};
  std::vector<double> x_pos {};
  std::vector<double> y_pos {};
  std::vector<double> yaw {};
  std::vector<double> curvature {};
  std::vector<double> first_derivative_x {};
  std::vector<double> first_derivative_y {};
  std::vector<double> second_derivative_x {};
  std::vector<double> second_derivative_y {};
};
  
  CubicSpline2D::CubicSpline2D(const std::vector<double>& x,
                               const std::vector<double>& y,
                               const std::size_t& interpolation_size){
    s(x, y);
    CubicSpline csplinex(m_s, x, interpolation_size);
    CubicSpline cspliney(m_s, y, interpolation_size);

    x_pos = csplinex.getPosition();
    y_pos = cspliney.getPosition();
    first_derivative_x = csplinex.getFirstDerivative();
    first_derivative_y = cspliney.getFirstDerivative();
    second_derivative_x = csplinex.getSecondDerivative();
    second_derivative_y = cspliney.getSecondDerivative();
    m_s_interval = csplinex.getInterval();

    std::transform(first_derivative_x.begin(),
                   first_derivative_x.end(),
                   first_derivative_y.begin(),
                   std::back_inserter(yaw),
                   [](double x, double y){
                     return std::atan2(y, x);
                   });
    computeCurvature();
  }

  void CubicSpline2D::s(const std::vector<double>& x,
          const std::vector<double>& y) {
    std::vector<double> dx;
    std::adjacent_difference(x.cbegin(),
                             x.cend(),
                             std::back_inserter(dx));
    std::vector<double> dy;
    std::adjacent_difference(y.cbegin(),
                             y.cend(),
                             std::back_inserter(dy));
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    std::transform(dy.cbegin(),
                   dy.cend(),
                   dx.cbegin(),
                   std::back_inserter(m_s),
                   [](double yy, double xx){
                     return std::hypot(xx, yy);
                   });

    std::partial_sum(m_s.begin(),
                     m_s.end(),
                     m_s.begin());
  }

  void CubicSpline2D::computeCurvature() {
    std::vector<double> ddydx;
    std::transform(second_derivative_y.begin(),
                   second_derivative_y.end(),
                   first_derivative_x.begin(),
                   std::back_inserter(ddydx),
                   std::multiplies<double>());

    std::vector<double> ddxdy;
    std::transform(second_derivative_x.begin(),
                   second_derivative_x.end(),
                   first_derivative_y.begin(),
                   std::back_inserter(ddxdy),
                   std::multiplies<double>());

    std::vector<double> diff_ddydx_ddxdy;
    std::transform(ddydx.begin(),
                   ddydx.end(),
                   ddxdy.begin(),
                   std::back_inserter(diff_ddydx_ddxdy),
                   std::minus<double>());

    std::vector<double> dxdxdydy;
    std::transform(first_derivative_x.begin(),
                   first_derivative_x.end(),
                   first_derivative_y.begin(),
                   std::back_inserter(dxdxdydy),
                   [](double x, double y){
                     return x*x + y*y;
                   });
    std::transform(diff_ddydx_ddxdy.begin(),
                   diff_ddydx_ddxdy.end(),
                   dxdxdydy.begin(),
                   std::back_inserter(curvature),
                   std::divides<double>());
  }
}

#endif /* CUBIC_SPLINE_INTERPOLATION_2D_H */
