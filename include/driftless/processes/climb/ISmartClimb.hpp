#ifndef __I_SMART_CLIMB_HPP__
#define __I_SMART_CLIMB_HPP__

#include <memory>

#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for process management
/// @author Matthew Backman
namespace processes {

/// @brief Namespace for the climb process
/// @author Matthew Backman
namespace climb {

/// @brief Interface for a generic smart climb process
/// @author Matthew Backman
class ISmartClimb {
 public:
  /// @brief Destroys the current smart climb object
  virtual ~ISmartClimb() = default;

  /// @brief Initializes the smart climb
  virtual void init() = 0;

  /// @brief Runs the smart climb
  virtual void run() = 0;

  /// @brief Pauses the smart climb
  virtual void pause() = 0;

  /// @brief Resumes the smart climb
  virtual void resume() = 0;

  /// @brief Starts the climb process
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  virtual void startClimb(std::shared_ptr<robot::Robot>& robot) = 0;

  /// @brief Checks if the smart climb process is paused
  /// @return __bool__ True if the smart climb is paused, false otherwise
  virtual bool isPaused() = 0;
};
}  // namespace climb
}  // namespace processes
}  // namespace driftless

#endif