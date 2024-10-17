#ifndef __BEZIER_CURVE_HPP__
#define __BEZIER_CURVE_HPP__

#include <cmath>
#include <cstdint>
#include <vector>

#include "pvegas/control/Point.hpp"
#include "pvegas/utils/UtilityFunctions.hpp"

namespace pvegas {
namespace control {
namespace path {
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

  // constructs a new bezier curve
  BezierCurve();

  // copies a bezier curve
  BezierCurve(const BezierCurve& bezierCurve) = default;

  // moves a bezier curve
  BezierCurve(BezierCurve&& bezierCurve) = default;

  // construct a new bezier curve with set points
  BezierCurve(Point c0, Point c1, Point c2, Point c3, Point c4, Point c5);

  // gets the point on the curve at time t
  Point getPointAt(double t);

  // copies a given bezier curve
  BezierCurve& operator=(BezierCurve& rhs) = default;

  // moves a given bezier curve
  BezierCurve& operator=(BezierCurve&& rhs) = default;
};
}  // namespace path
}  // namespace control
}  // namespace pvegas
#endif