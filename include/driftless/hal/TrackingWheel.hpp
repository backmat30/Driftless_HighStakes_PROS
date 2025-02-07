#ifndef __TRACKING_WHEEL_HPP__
#define __TRACKING_WHEEL_HPP__

#include <memory>

#include "driftless/io/IDistanceTracker.hpp"
#include "driftless/io/IRotationSensor.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for the hardware abstraction layer
/// @author Matthew Backman
namespace hal {

/// @brief Class representing a tracking wheel
/// @author Matthew Backman
class TrackingWheel : public driftless::io::IDistanceTracker {
 private:
  // the rotation sensor for the wheel
  std::unique_ptr<driftless::io::IRotationSensor> m_rotation_sensor{};

  // the radius of the wheel
  double m_wheel_radius{};

 public:
  /// @brief Constructs a new tracking wheel
  /// @param rotation_sensor __std::unique_ptr<io::IRotationSensor>&__ The
  /// rotation sensor used in the tracking wheel
  /// @param wheel_radius __double__ The radius of the tracking wheel, in inches
  TrackingWheel(
      std::unique_ptr<driftless::io::IRotationSensor>& rotation_sensor,
      double wheel_radius);

  /// @brief Initializes the tracking wheel
  void init() override;

  /// @brief Resets the tracking wheel
  void reset() override;

  /// @brief Gets the distance travelled by the tracking wheel, in inches
  /// @return __double__ The distance
  double getDistance() override;

  /// @brief Sets the distance on the tracking wheel
  /// @param distance __double__ The new distance
  void setDistance(double distance) override;
};
}  // namespace hal
}  // namespace driftless
#endif