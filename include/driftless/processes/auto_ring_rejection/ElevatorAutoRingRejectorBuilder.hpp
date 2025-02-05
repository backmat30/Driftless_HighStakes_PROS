#ifndef __ELEVATOR_AUTO_RING_REJECTOR_BUILDER_HPP__
#define __ELEVATOR_AUTO_RING_REJECTOR_BUILDER_HPP__

#include "driftless/processes/auto_ring_rejection/ElevatorAutoRingRejector.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for process management
/// @author Matthew Backman
namespace processes {

/// @brief Namespace for the auto ring rejection process
/// @author Matthew Backman
namespace auto_ring_rejection {

/// @brief Builder for the ElevatorAutoRingRejector class
/// @author Matthew Backman
class ElevatorAutoRingRejectorBuilder {
 private:
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

 public:
  /// @brief Add a delayer to the builder
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to add
  /// @return __ElevatorAutoRingRejectorBuilder*__ Pointer to the current
  /// builder
  ElevatorAutoRingRejectorBuilder* withDelayer(
      std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Add a mutex to the builder
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to add
  /// @return __ElevatorAutoRingRejectorBuilder*__ Pointer to the current
  /// builder
  ElevatorAutoRingRejectorBuilder* withMutex(
      std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Add a task to the builder
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to add
  /// @return __ElevatorAutoRingRejectorBuilder*__ Pointer to the current
  /// builder
  ElevatorAutoRingRejectorBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Builds a new auto ring rejector using the values in the builder
  /// @return __unique_ptr<ElevatorAutoRingRejector>__ Pointer to the new auto
  /// ring rejector
  std::unique_ptr<ElevatorAutoRingRejector> build();
};
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless
#endif