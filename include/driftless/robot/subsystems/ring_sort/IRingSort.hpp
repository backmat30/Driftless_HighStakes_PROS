#ifndef __I_RING_SORT_HPP__
#define __I_RING_SORT_HPP__

#include "driftless/io/RGBValue.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace ring_sort {
class IRingSort {
 public:
  /// @brief Deletes the ring sort object
  virtual ~IRingSort() = default;

  /// @brief Initializes the ring sorter
  virtual void init() = 0;

  /// @brief Runs the ring sorter
  virtual void run() = 0;

  /// @brief Gets the hue from the optical sensor
  /// @return The hue value of the sensor
  virtual double getRingHue() = 0;

  /// @brief Gets the RGB value of the ring from the optical sensor
  /// @return The RGB value of the ring
  virtual io::RGBValue getRingRGB() = 0;

  /// @brief Gets the distance to the end of the elevator
  /// @return __double__ distance to the end of the elevator
  virtual double getDistanceToElevatorEnd() = 0;

  /// @brief Determines if there is a ring in front of the sensor
  /// @return __True__ if there is a ring, otherwise false
  virtual bool hasRing() = 0;
};
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif