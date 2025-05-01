#ifndef __PISTON_RING_REJECTION_HPP__
#define __PISTON_RING_REJECTION_HPP__

#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/elevator/IRingRejection.hpp"

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

/// @brief Class representing the piston ring rejection system
/// @author Matthew Backman
class PistonRingRejection : public IRingRejection {
 private:
  /// @brief The pistons controlling the left side of the rejection system
  driftless::hal::PistonGroup m_left_pistons{};

    /// @brief The pistons controlling the right side of the rejection system
  driftless::hal::PistonGroup m_right_pistons{};

  ERejectionDirection rejection_direction{ERejectionDirection::FORWARD};

 public:
  /// @brief Initializes the ring rejector
  void init() override;

  /// @brief Runs the ring rejector
  void run() override;

  /// @brief Deploys the ring rejector
  void deploy() override;

  /// @brief Retracts the ring rejector
  void retract() override;

  /// @brief Sets the direction the ring rejector will send rings
  /// @param direction __ERejectionDirection__ The direction to send the rings
  void setDeploymentDirection(ERejectionDirection direction) override;

  /// @brief Determines if the ring rejector is actively deployed
  /// @return __True__ if extended, __false__ otherwise
  bool isDeployed() override;

  /// @brief Sets the pistons used by the left side of the rejection system
  /// @param pistons The new pistons used
  void setLeftPistons(driftless::hal::PistonGroup& pistons);

  /// @brief Sets the pistons used by the right side of the rejection system
  /// @param pistons The new pistons used
  void setRightPistons(driftless::hal::PistonGroup& pistons);
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif