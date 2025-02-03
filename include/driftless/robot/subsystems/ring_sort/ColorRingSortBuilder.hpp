#ifndef __COLOR_RING_SORT_BUILDER_HPP__
#define __COLOR_RING_SORT_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/ring_sort/ColorRingSort.hpp"

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

/// @brief The builder for the ColorRingSort class
/// @author Matthew Backman
class ColorRingSortBuilder {
 private:
  std::unique_ptr<driftless::io::IColorSensor> m_color_sensor{};

  double m_max_ring_distance{};

  double m_distance_to_elevator_end{};

 public:
  /// @brief Adds a color sensor to the builder
  /// @param color_sensor __std::unique_ptr<driftless::io::IColorSensor>&__ The
    /// color sensor
  /// @return __ColorRingSortBuilder*__ Pointer to the current builder
  ColorRingSortBuilder* withColorSensor(
      std::unique_ptr<driftless::io::IColorSensor>& color_sensor);

  /// @brief Adds a max ring distance to the builder
  /// @param max_ring_distance __double__ The max ring distance
  /// @return __ColorRingSortBuilder*__ Pointer to the current builder
  ColorRingSortBuilder* withMaxRingDistance(double max_ring_distance);

  /// @brief Adds a distance to the end of the elevator to the builder
  /// @param distance_to_elevator_end __double__ The distance
  /// @return __ColorRingSortBuilder*__ Pointer to the current builder
  ColorRingSortBuilder* withDistanceToElevatorEnd(
      double distance_to_elevator_end);

  /// @brief Builds a new ColorRingSort object using the given values
  /// @return __std::unique_ptr<IRingSort>__ Pointer to the new ColorRingSort
  std::unique_ptr<IRingSort> build();
};
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif