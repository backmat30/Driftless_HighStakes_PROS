#ifndef __PROS_DISTANCE_SENSOR_HPP__
#define __PROS_DISTANCE_SENSOR_HPP__

#include <memory>

#include "driftless/io/IDistanceSensor.hpp"
#include "pros/distance.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief Adapter class for the PROS distance sensor
/// @author Matthew Backman
class ProsDistanceSensor : public driftless::io::IDistanceSensor {
 private:
  // conversion factor from mm to in
  static constexpr double MM_TO_INCHES{1.0 / 25.4};

  // sensor being adapted
  std::unique_ptr<pros::Distance> m_distance_sensor{};

  // tuning constant for readings to improve accuracy
  double m_tuning_constant{};

  // distance offset for the sensor
  double m_tuning_offset{};

 public:
  /// @brief Constructor
  /// @param distance_sensor __std::unique_ptr<pros::Distance>&__ The distance
  /// sensor to adapt
  /// @param tuning_constant __double__ The tuning constant for readings
  /// (default is 1)
  /// @param tuning_offset __double__ The distance offset for the sensor
  /// (default is 0)
  ProsDistanceSensor(std::unique_ptr<pros::Distance>& distance_sensor,
                     double tuning_constant = 1, double tuning_offset = 0);

  /// @brief Initializes the distance sensor
  void init() override;

  /// @brief Resets the distance sensor
  void reset() override;

  /// @brief Gets the distance from the sensor
  /// @return __double__ The distance measured by the sensor
  double getDistance() override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif