#ifndef __I_AUTO_RING_REJECTOR_HPP__
#define __I_AUTO_RING_REJECTOR_HPP__

#include <memory>

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/robot/Robot.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for process management
/// @author Matthew Backman
namespace processes {

/// @brief Namespace for the auto ring rejection process
/// @author Matthew Backman
namespace auto_ring_rejection {

/// @brief Interface for a generic automatic ring rejection process
/// @author Matthew Backman
class IAutoRingRejector {
 public:
  /// @brief Destroys the current auto ring rejector object
  virtual ~IAutoRingRejector() = default;

  /// @brief Initializes the auto ring rejector
  virtual void init() = 0;

  /// @brief Runs the auto ring rejector
  virtual void run() = 0;

  /// @brief Pauses the auto ring rejector
  virtual void pause() = 0;

  /// @brief Resumes the auto ring rejector
  virtual void resume() = 0;

  /// @brief Rejects rings
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot object
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The alliance
  /// object
  virtual void rejectRings(std::shared_ptr<robot::Robot>& robot,
                           std::shared_ptr<alliance::IAlliance>& alliance) = 0;

  /// @brief Checks if the auto ring rejector is paused
  /// @return __bool__ True if the auto ring rejector is paused, false otherwise
  virtual bool isPaused() = 0;
};
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless
#endif