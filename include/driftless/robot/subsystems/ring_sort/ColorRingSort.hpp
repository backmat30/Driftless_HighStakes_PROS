#ifndef __COLOR_RING_SORT_HPP__
#define __COLOR_RING_SORT_HPP__

#include <cstdint>
#include <memory>

#include "driftless/io/IColorSensor.hpp"
#include "driftless/robot/subsystems/ring_sort/IRingSort.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace ring_sort {
class ColorRingSort : public IRingSort {
 private:
  /// @brief The color sensor used to detect rings
  std::unique_ptr<driftless::io::IColorSensor> m_color_sensor{};

  /// @brief The maximum distance between the sensor and a ring
  double m_max_ring_distance{};

  /// @brief The distance from the color sensor to the end of the elevator
  double m_distance_to_elevator_end{};

 public:
  /// @brief Initializes the color ring sorter
  void init() override;

  /// @brief Runs the color ring sorter
  void run() override;

  /// @brief Gets the hue from the optical sensor
  /// @return The hue value of the sensor
  double getRingHue() override;

  /// @brief Gets the RGB value of the ring from the optical sensor
  /// @return The RGB value of the ring
  io::RGBValue getRingRGB() override;

  /// @brief Gets the distance to the end of the elevator
  /// @return __double__ distance to the end of the elevator
  double getDistanceToElevatorEnd() override;

  /// @brief Determines if there is a ring in front of the sensor
  /// @return __True__ if there is a ring, otherwise false
  bool hasRing() override;

  /// @brief Sets the color sensor the sorter will use
  /// @param color_sensor The new color sensor
  void setColorSensor(
      std::unique_ptr<driftless::io::IColorSensor>& color_sensor);

  /// @brief Sets the maximum object distance to be considered a ring
  /// @param max_ring_distance The new max distance
  void setMaxRingDistance(double max_ring_distance);

  /// @brief Sets the distance to the end of the elevator
  /// @param distance_to_elevator_end The distance from the sensor to the end of
  /// the elevator
  void setDistanceToElevatorEnd(double distance_to_elevator_end);
};
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif