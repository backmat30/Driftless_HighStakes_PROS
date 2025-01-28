#ifndef __AUTO_RING_REJECTION_PROCESS_HPP__
#define __AUTO_RING_REJECTION_PROCESS_HPP__

#include "driftless/processes/AProcess.hpp"
#include "driftless/processes/auto_ring_rejection/IAutoRingRejector.hpp"

namespace driftless {
namespace processes {
namespace auto_ring_rejection {
class AutoRingRejectionProcess : public AProcess {
 private:
  std::unique_ptr<auto_ring_rejection::IAutoRingRejector> m_ring_rejector{};

 public:
  AutoRingRejectionProcess(
      std::unique_ptr<auto_ring_rejection::IAutoRingRejector>& ring_rejector);

  void init() override;

  void run() override;

  void pause() override;

  void resume() override;

  void command(EProcessCommand command_name, va_list& args) override;

  void* state(EProcessState state_name) override;
};
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless
#endif