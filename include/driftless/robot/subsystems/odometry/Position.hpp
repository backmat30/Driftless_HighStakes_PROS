#ifndef __POSITION_HPP__
#define __POSITION_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

/// @brief Struct representing the position and velocity of the robot
/// @author Matthew Backman
struct Position {
  // x coordinate
  double x{};

  // y coordinate
  double y{};

  // angle
  double theta{};

  // x velocity
  double xV{};

  // y velocity
  double yV{};

  // angular velocity
  double thetaV{};
};

}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif