#ifndef __BEZIER_CURVE_INTERPOLATION_HPP__
#define __BEZIER_CURVE_INTERPOLATION_HPP__

#include <vector>

#include "driftless/control/path/BezierCurve.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the path follower control
/// @author Matthew Backman
namespace path {

/// @brief Class representing a set of interpolated quintic bezier curves
/// @author Matthew Backman
class BezierCurveInterpolation {
 public:
  /// @brief Calculates the points along the curves using a set of control
  /// points
  /// @param control_points __std::vector<Point>&__ The control points used for
  /// the bezier curves. must fit (n - 2) % 3 = 0
  /// @return __std::vector<Point>&__ The points along the interpolated bezier
  /// curve
  static std::vector<Point> calculate(std::vector<Point>& control_points);
};
}  // namespace path
}  // namespace control
}  // namespace driftless
#endif