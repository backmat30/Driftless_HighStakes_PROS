#ifndef __I_AUTO_RING_REJECTOR_HPP__
#define __I_AUTO_RING_REJECTOR_HPP__

#include <memory>

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/robot/Robot.hpp"

namespace driftless {
namespace processes {
namespace auto_ring_rejection {
class IAutoRingRejector {
 public:
  virtual ~IAutoRingRejector() = default;

  virtual void init() = 0;

  virtual void run() = 0;

  virtual void pause() = 0;

  virtual void resume() = 0;

  virtual void rejectRings(std::shared_ptr<robot::Robot>& robot,
                           std::shared_ptr<alliance::IAlliance>& alliance) = 0;

  virtual bool isPaused() = 0;
};
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless
#endif