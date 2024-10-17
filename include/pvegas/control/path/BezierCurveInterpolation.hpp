#ifndef __BEZIER_CURVE_INTERPOLATION_HPP__
#define __BEZIER_CURVE_INTERPOLATION_HPP__

#include <vector>

#include "pvegas/control/path/BezierCurve.hpp"

namespace pvegas {
namespace control {
namespace path {
class BezierCurveInterpolation {
 public:
  // creates bezier curves to route through all control points
  static std::vector<Point> calculate(std::vector<Point>& control_points);
};
}  // namespace path
}  // namespace control
}  // namespace pvegas
#endif