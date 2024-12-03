#include "driftless/control/path/BezierCurveInterpolation.hpp"

namespace driftless {
namespace control {
namespace path {
std::vector<Point> BezierCurveInterpolation::calculate(
    std::vector<Point>& control_points) {
  // if the control set is invalid, return an empty vector
  if (control_points.size() == 0 || (control_points.size() - 1) % 3 != 0) {
    return std::vector<Point>{};
  }

  std::vector<BezierCurve> bezier_curves{};
  for (int i{}; i < control_points.size() - 1; i += 3) {
    // add a new curve to the list with empty points in the unknown positions
    bezier_curves.push_back(
        BezierCurve{control_points[i], Point{}, control_points[i + 1],
                    control_points[i + 2], Point{}, control_points[i + 3]});
  }

  // calculate first smoothing point for the first curve
  bezier_curves[0].k1 = (bezier_curves[0].k0 + bezier_curves[0].k2) / 2.0;

  // calculate first smoothing point for other curves
  for (int i{1}; i < bezier_curves.size(); ++i) {
    bezier_curves[i].k1 = ((bezier_curves[i].k0 * 4.0) -
                           bezier_curves[i - 1].k3 + bezier_curves[i].k2) /
                          4.0;
  }
  // calculate last smoothing point for curves other than last
  for (int i{}; i < bezier_curves.size() - 1; ++i) {
    bezier_curves[i].k4 = bezier_curves[i].k5 * 2.0 - bezier_curves[i + 1].k1;
  }
  // calculate last smoothing point for last curve
  bezier_curves[bezier_curves.size() - 1].k4 =
      (bezier_curves[bezier_curves.size() - 1].k5 +
       bezier_curves[bezier_curves.size() - 1].k3) /
      2.0;

  // calculate points along the line
  std::vector<Point> result{};
  for(BezierCurve& curve : bezier_curves) {
    for(double t{0.0}; t < 1.0; t += 0.02) {
      result.push_back(curve.getPointAt(t));
    }
  }
  return result;
}
}  // namespace path
}  // namespace control
}  // namespace driftless