#ifndef __RING_SORT_SUBSYSTEM_HPP__
#define __RING_SORT_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/ring_sort/IRingSort.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace ring_sort {
/// @brief Subsystem for the ring sorter
class RingSortSubsystem : public ASubsystem {
 private:
  std::unique_ptr<IRingSort> m_ring_sort{};

 public:
  /// @brief Creates a new RingSortSubsystem
  /// @param ring_sort The ring sorter to use
  RingSortSubsystem(std::unique_ptr<IRingSort>& ring_sort);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Sends a command to the subsystem
  /// @param command_name The name of the command
  /// @param args Possible arguments for the desired command
  void command(ESubsystemCommand command_name, va_list& args) override;

  /// @brief Gets a specified state of the subsystem
  /// @param state_name The desired state
  /// @return __void*__ Pointer to the value of the state
  void* state(ESubsystemState state_name) override;
};
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif