#ifndef __I_POSITION_RESETTER_HPP__
#define __I_POSITION_RESETTER_HPP__

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

/// @brief Interface to reset positions to absolute field position
/// @author Matthew Backman
class IPositionResetter {
 public:
  /// @brief Destructor
  virtual ~IPositionResetter() = default;

  /// @brief Initializes the position resetter
  virtual void init() = 0;

  /// @brief Runs the position resetter
  virtual void run() = 0;

  /// @brief Retrieves the x portion of the reset position
  /// @param theta __double__ The angle to use for the calculation
  /// @return __double__ The x portion of the reset position
  virtual double getResetX(double theta) = 0;

  /// @brief Retrieves the y portion of the reset position
  /// @param theta __double__ The angle to use for the calculation
  /// @return __double__ The y portion of the reset position
  virtual double getResetY(double theta) = 0;

  /// @brief Gets the raw value of the resetter
  /// @return __double__ The raw value
  virtual double getRawValue() = 0;
};

}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif