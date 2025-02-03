#ifndef __PISTON_RING_REJECTION_BUILDER_HPP__
#define __PISTON_RING_REJECTION_BUILDER_HPP__

#include "driftless/robot/subsystems/elevator/PistonRingRejection.hpp"

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

/// @brief Builder class for creating PistonRingRejection objects
/// @author Matthew Backman
class PistonRingRejectionBuilder {
 private:
  driftless::hal::PistonGroup m_pistons{};

 public:
  /// @brief Adds a piston to the builder
  /// @param piston The piston to add
  /// @return __PistonRingRejectionBuilder*__ Pointer to the current builder
  PistonRingRejectionBuilder* withPiston(
      std::unique_ptr<driftless::io::IPiston>& piston);

  /// @brief Builds a new PistonRingRejection object
  /// @return __std::unique_ptr<PistonRingRejection>__ Pointer to the new
  /// PistonRingRejection object
  std::unique_ptr<IRingRejection> build();
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif