#ifndef __E_REJECTION_DIRECTION_HPP__
#define __E_REJECTION_DIRECTION_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for elevator subsystem code
/// @author Matthew Backman
namespace elevator {

/// @brief Enumeration representing a direction the color sort can reject a ring
/// @author Matthew Backman
enum class ERejectionDirection { LEFT, RIGHT};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif