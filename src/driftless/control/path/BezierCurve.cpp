#include "driftless/control/path/BezierCurve.hpp"

namespace driftless {
namespace control {
namespace path {
BezierCurve::BezierCurve(Point c0, Point c1, Point c2, Point c3, Point c4,
                         Point c5)
    : k0{c0}, k1{c1}, k2{c2}, k3{c3}, k4{c4}, k5{c5} {}

Point BezierCurve::getPointAt(double t) {
  return (k0 * std::pow(1 - t, 5)) +
         (k1 * binomialCoefficient(5, 1) * std::pow(1 - t, 4) * t) +
         (k2 * binomialCoefficient(5, 2) * std::pow(1 - t, 3) *
          std::pow(t, 2)) +
         (k3 * binomialCoefficient(5, 3) * std::pow(1 - t, 2) *
          std::pow(t, 3)) +
         (k4 * binomialCoefficient(5, 4) * (1 - t) * std::pow(t, 4)) +
         (k5 * std::pow(t, 5));
}
}  // namespace path
}  // namespace control
}  // namespace driftless