#ifndef __BEZIER_CURVE_HPP__
#define __BEZIER_CURVE_HPP__

#include <cmath>
#include <cstdint>
#include <vector>

#include "driftless/control/Point.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the path follower control
/// @author Matthew Backman
namespace path {

/// @brief Class representing a quintic bezier curve
/// @author Matthew Backman
class BezierCurve {
 public:
  // first control point
  Point k0{};

  // second control point
  Point k1{};

  // third control point
  Point k2{};

  // fourth control point
  Point k3{};

  // fifth control point
  Point k4{};

  // sixth control point
  Point k5{};

  /// @brief Constructs a new bezier curve
  BezierCurve();

  /// @brief Constructs a new bezier curve
  /// @param bezierCurve __const BezierCurve&__ The bezier curve to copy
  BezierCurve(const BezierCurve& bezierCurve) = default;

  /// @brief Constructs a new bezier curve
  /// @param bezierCurve __BezierCurve&&__ The bezier curve to move
  BezierCurve(BezierCurve&& bezierCurve) = default;

  /// @brief Constructs a new bezier curve
  /// @param c0 __Point__ The first control point
  /// @param c1 __Point__ The second control point
  /// @param c2 __Point__ The third control point
  /// @param c3 __Point__ The fourth control point
  /// @param c4 __Point__ The fifth control point
  /// @param c5 __Point__ The sixth control point
  BezierCurve(Point c0, Point c1, Point c2, Point c3, Point c4, Point c5);

  /// @brief Gets the point at time t
  /// @param t __double__ The t value of the parametric, between 0 and 1
  /// @return __Point__ The point at the time t
  Point getPointAt(double t);

  /// @brief Copies a bezier curve
  /// @param rhs __BezierCurve&__ The bezier curve to copy
  /// @return __BezierCurve&__ The copy of the bezier curve
  BezierCurve& operator=(BezierCurve& rhs) = default;

  /// @brief Moves a bezier curve
  /// @param rhs __BezierCurve&&__ The bezier curve being moved
  /// @return __BezierCurve&__ Reference to the moved bezier curve
  BezierCurve& operator=(BezierCurve&& rhs) = default;
};
}  // namespace path
}  // namespace control
}  // namespace driftless
#endif