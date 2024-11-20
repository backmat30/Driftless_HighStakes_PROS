#ifndef __PISTON_RING_REJECTION_HPP__
#define __PISTON_RING_REJECTION_HPP__

#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/elevator/IRingRejection.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
class PistonRingRejection : public IRingRejection {
 private:
  /// @brief The pistons controlling the rejection system
  driftless::hal::PistonGroup m_pistons{};

 public:
  /// @brief Initializes the ring rejector
  void init() override;

  /// @brief Runs the ring rejector
  void run() override;

  /// @brief Deploys the ring rejector
  void deploy() override;

  /// @brief Retracts the ring rejector
  void retract() override;

  /// @brief Determines if the ring rejector is actively deployed
  /// @return __True__ if extended, __false__ otherwise
  bool isDeployed() override;

  /// @brief Sets the pistons used by the rejection system
  /// @param pistons The new pistons used
  void setPistons(driftless::hal::PistonGroup& pistons);
};
}  // namespace elevator
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif