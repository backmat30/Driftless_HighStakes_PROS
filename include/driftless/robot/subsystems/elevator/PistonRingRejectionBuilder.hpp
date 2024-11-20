#ifndef __PISTON_RING_REJECTION_BUILDER_HPP__
#define __PISTON_RING_REJECTION_BUILDER_HPP__

#include "driftless/robot/subsystems/elevator/PistonRingRejection.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
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