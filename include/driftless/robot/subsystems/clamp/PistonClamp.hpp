#ifndef __PISTON_CLAMP_HPP__
#define __PISTON_CLAMP_HPP__

#include <memory>

#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/clamp/IClamp.hpp"

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

/// @brief Class representing a piston-controlled clamp
/// @author Matthew Backman
class PistonClamp : public IClamp {
 private:
  // the pistons controlling the clamp
  driftless::hal::PistonGroup m_pistons{};

  // whether the clamp is active or idle
  bool state{};

 public:
  /// @brief Initializes the clamp
  void init() override;

  /// @brief Runs the clamp
  void run() override;

  /// @brief Sets the state of the clamp
  /// @param clamped __bool__ The state to set (true for clamped, false for unclamped)
  void setState(bool clamped) override;

  /// @brief Gets the state of the clamp
  /// @return __bool__ The current state of the clamp
  bool getState() override;

  /// @brief Sets the pistons used by the clamp
  /// @param pistons __driftless::hal::PistonGroup&__ The pistons to set
  void setPistons(driftless::hal::PistonGroup& pistons);
};

}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif