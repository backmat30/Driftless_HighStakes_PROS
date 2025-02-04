#ifndef __PISTON_CLAMP_BUILDER_HPP__
#define __PISTON_CLAMP_BUILDER_HPP__

#include "driftless/robot/subsystems/clamp/PistonClamp.hpp"

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

/// @brief Builder class for creating PistonClamp objects
/// @author Matthew Backman
class PistonClampBuilder {
 private:
  // piston group used to build the clamp
  driftless::hal::PistonGroup m_pistons{};

 public:
  /// @brief Adds a piston to the builder
  /// @param piston __std::unique_ptr<driftless::io::IPiston>&__ The piston to add
  /// @return __PistonClampBuilder*__ The builder instance
  PistonClampBuilder* withPiston(std::unique_ptr<driftless::io::IPiston>& piston);

  /// @brief Builds a PistonClamp object
  /// @return __std::unique_ptr<PistonClamp>__ The built PistonClamp object
  std::unique_ptr<PistonClamp> build();
};

}  // namespace clamp
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif