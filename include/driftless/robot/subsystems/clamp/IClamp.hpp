#ifndef __I_CLAMP_HPP__
#define __I_CLAMP_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the clamp subsystem
/// @author Matthew Backman
namespace clamp {

/// @brief Interface for clamp objects
/// @author Matthew Backman
class IClamp {
 public:
  /// @brief Destroys the clamp object
  virtual ~IClamp() = default;

  /// @brief Initializes the clamp
  virtual void init() = 0;

  /// @brief Runs the clamp
  virtual void run() = 0;

  /// @brief Sets the state of the clamp
  /// @param clamped __bool__ The state to set (true for clamped, false for
  /// unclamped)
  virtual void setState(bool clamped) = 0;

  /// @brief Gets the state of the clamp
  /// @return __bool__ The current state of the clamp
  virtual bool getState() = 0;

  /// @brief Determines if the clamp has a goal
  /// @return __bool__ True if the clamp has a goal, false otherwise
  virtual bool hasGoal() = 0;
};

}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif