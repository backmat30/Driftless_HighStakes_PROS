#ifndef __PROS_DISTANCE_SENSOR_HPP__
#define __PROS_DISTANCE_SENSOR_HPP__

#include <memory>

#include "pros/distance.hpp"
#include "pvegas/io/IDistanceSensor.hpp"

namespace pvegas {
namespace pros_adapters {
class ProsDistanceSensor : public pvegas::io::IDistanceSensor {
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
  // constructor
  ProsDistanceSensor(std::unique_ptr<pros::Distance>& distance_sensor, double tuning_constant = 1, double tuning_offset = 0);

  // initialize the distance sensor
  void init() override;

  // resets the distance sensor
  void reset() override;

  // gets the distance from the sensor
  double getDistance() override;
};
}  // namespace pros_adapters
}  // namespace pvegas
#endif