#ifndef __E_TURN_DIRECTION_HPP__
#define __E_TURN_DIRECTION_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for basic motion control algorithms
/// @author Matthew Backman
namespace motion {

/// @brief directions the robot can turn in
/// @details - __AUTO__ goes in the direction with the shortest path
/// - __CLOCKWISE__ goes in the _negative_ direction
/// - __COUNTERCLOCKWISE__ goes in the _positive_ direction
/// @author Matthew Backman
enum class ETurnDirection { AUTO, CLOCKWISE, COUNTERCLOCKWISE };
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif