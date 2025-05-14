#ifndef __I_RING_REJECTION_HPP__
#define __I_RING_REJECTION_HPP__

#include "driftless/robot/subsystems/elevator/ERejectionDirection.hpp"
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

/// @brief Interface for ring rejection systems
/// @author Matthew Backman
class IRingRejection {
 public:
  /// @brief Deletes the ring rejection object
  virtual ~IRingRejection() = default;

  /// @brief Initializes the ring rejector
  virtual void init() = 0;

  /// @brief Runs the ring rejector
  virtual void run() = 0;

  /// @brief Deploys the rejection system
  virtual void deploy() = 0;

  /// @brief Retracts the rejection system
  virtual void retract() = 0;

  /// @brief Sets the direction the ring rejector will send rings
  /// @param direction __ERejectionDirection__ The direction to send the rings
  virtual void setDeploymentDirection(ERejectionDirection direction) = 0;

  /// @brief Determines if the ring rejector is actively deployed
  /// @return __True__ if extended, __false__ otherwise
  virtual bool isDeployed() = 0;
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif