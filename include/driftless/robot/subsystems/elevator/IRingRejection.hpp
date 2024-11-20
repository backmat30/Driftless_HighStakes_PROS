#ifndef __I_RING_REJECTION_HPP__
#define __I_RING_REJECTION_HPP__

namespace driftless {
namespace robot {
namespace subsystems {
namespace elevator {
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

  /// @brief Determines if the ring rejector is actively deployed
  /// @return __True__ if extended, __false__ otherwise
  virtual bool isDeployed() = 0;
};
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif