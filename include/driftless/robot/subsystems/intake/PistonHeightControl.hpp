#ifndef __PISTON_HEIGHT_CONTROL_HPP__
#define __PISTON_HEIGHT_CONTROL_HPP__

#include <memory>

#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/intake/IHeightControl.hpp"

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

/// @brief Class for controlling the height of the intake using pistons
/// @author Matthew Backman
class PistonHeightControl : public IHeightControl {
 private:
  // the group of pistons being used
  driftless::hal::PistonGroup m_height_pistons{};

  hal::PistonGroup m_secondary_pistons{};

  // whether the intake is up or down
  bool raised{false};

 public:
  /// @brief Initializes the height controller
  void init() override;

  /// @brief Runs the height controller
  void run() override;

  /// @brief Sets the height of the intake
  /// @param up __bool__ Whether to raise or lower the intake
  void setHeight(bool up) override;

  /// @brief Pulls the intake in
  void pullIn() override;

  /// @brief Pushes the intake out
  void pushOut() override;

  /// @brief Gets the height of the intake
  /// @return __bool__ Whether the intake is raised
  bool isRaised() override;

  /// @brief Sets the pistons for height control
  /// @param pistons __driftless::hal::PistonGroup&__ The pistons to use
  void setHeightPistons(driftless::hal::PistonGroup& pistons);

  /// @brief Sets the pistons to push forward/pull back the intake
  /// @param pistons __hal::PistonGroup&__ The pistons to use
  void setSecondaryPistons(driftless::hal::PistonGroup& pistons);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif