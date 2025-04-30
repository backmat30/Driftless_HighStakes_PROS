#ifndef __PISTON_HEIGHT_CONTROL_BUILDER_HPP__
#define __PISTON_HEIGHT_CONTROL_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/intake/PistonHeightControl.hpp"

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

/// @brief Builder class for creating PistonHeightControl objects
class PistonHeightControlBuilder {
 private:
  // the pistons used to build the height controller
  driftless::hal::PistonGroup m_height_pistons{};

  hal::PistonGroup m_secondary_pistons{};

 public:
  /// @brief Adds a height control piston to the builder
  /// @param piston __std::unique_ptr<driftless::io::IPiston>&__ The piston to add
  /// @return __PistonHeightControlBuilder*__ The builder instance
  PistonHeightControlBuilder* withHeightPiston(
      std::unique_ptr<driftless::io::IPiston>& piston);

  /// @brief Adds a secondary level piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>&__ The piston to add
  /// @return __PistonHeightControlBuilder*__ The builder instance
  PistonHeightControlBuilder* withSecondaryPiston(
      std::unique_ptr<driftless::io::IPiston>& piston);

  /// @brief Builds a new PistonHeightControl object
  /// @return __std::unique_ptr<PistonHeightControl>__ The built PistonHeightControl object
  std::unique_ptr<PistonHeightControl> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif