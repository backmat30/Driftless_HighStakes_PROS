#ifndef __I_RING_SORT_HPP__
#define __I_RING_SORT_HPP__

#include "driftless/io/RGBValue.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for ring sort code
/// @author Matthew Backman
namespace ring_sort {

/// @brief The interface for the ring sorter
/// @author Matthew Backman
class IRingSort {
 public:
  /// @brief Deletes the ring sort object
  virtual ~IRingSort() = default;

  /// @brief Initializes the ring sorter
  virtual void init() = 0;

  /// @brief Runs the ring sorter
  virtual void run() = 0;

  /// @brief Gets the hue from the optical sensor
  /// @return __double__ The hue value of the sensor
  virtual double getRingHue() = 0;

  /// @brief Gets the RGB value of the ring from the optical sensor
  /// @return __io::RGBValue__ The RGB value of the ring
  virtual io::RGBValue getRingRGB() = 0;

  /// @brief Gets the distance to the end of the elevator
  /// @return __double__ distance to the end of the elevator
  virtual double getDistanceToElevatorEnd() = 0;

  /// @brief Determines if there is a ring in front of the sensor
  /// @return __bool__ True if there is a ring, otherwise false
  virtual bool hasRing() = 0;
};
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif