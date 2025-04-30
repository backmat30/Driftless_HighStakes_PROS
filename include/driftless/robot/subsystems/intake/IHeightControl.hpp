#ifndef __I_HEIGHT_CONTROL_HPP__
#define __I_HEIGHT_CONTROL_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for intake subsystem code
/// @author Matthew Backman
namespace intake {

/// @brief Interface for height control mechanisms
/// @author Matthew Backman
class IHeightControl {
 public:
  /// @brief Destroys the height control object
  virtual ~IHeightControl() = default;

  /// @brief Initializes the height controller
  virtual void init() = 0;

  /// @brief Runs the height controller
  virtual void run() = 0;

  /// @brief Sets the height of the intake
  /// @param up __bool__ Whether to raise or lower the intake
  virtual void setHeight(bool up) = 0;

  /// @brief Pulls the intake in
  virtual void pullIn() = 0;

  /// @brief Pushes the intake out
  virtual void pushOut() = 0;

  /// @brief Gets the position of the intake
  /// @return __bool__ Whether the intake is raised
  virtual bool isRaised() = 0;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif