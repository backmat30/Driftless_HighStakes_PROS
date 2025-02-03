#ifndef __AUTO_RING_REJECTION_PROCESS_HPP__
#define __AUTO_RING_REJECTION_PROCESS_HPP__

#include "driftless/processes/AProcess.hpp"
#include "driftless/processes/auto_ring_rejection/IAutoRingRejector.hpp"

namespace driftless {
namespace processes {
namespace auto_ring_rejection {
class AutoRingRejectionProcess : public AProcess {
 private:
  /// @brief The auto ring rejector
  std::unique_ptr<auto_ring_rejection::IAutoRingRejector> m_ring_rejector{};

 public:
  /// @brief Constructs an AutoRingRejectionProcess object
  /// @param ring_rejector __std::unique_ptr<auto_ring_rejection::IAutoRingRejector>&__ The auto ring rejector
  AutoRingRejectionProcess(
      std::unique_ptr<auto_ring_rejection::IAutoRingRejector>& ring_rejector);

  /// @brief Initializes the process
  void init() override;

  /// @brief Runs the process
  void run() override;

  /// @brief Pauses the process
  void pause() override;

  /// @brief Resumes the process
  void resume() override;

  /// @brief Sends a command to the process
  /// @param command_name __EProcessCommand__ The command to send
  /// @param args __va_list&__ The arguments to the command
  void command(EProcessCommand command_name, va_list& args) override;

  /// @brief Gets the state of the process
  /// @param state_name __EProcessState__ The state to get
  /// @return __void*__ A pointer to the state
  void* state(EProcessState state_name) override;
};
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless
#endif