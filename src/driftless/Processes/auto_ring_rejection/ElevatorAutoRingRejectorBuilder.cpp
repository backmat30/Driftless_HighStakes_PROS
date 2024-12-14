#include "driftless/processes/auto_ring_rejection/ElevatorAutoRingRejectorBuilder.hpp"

namespace driftless {
namespace processes {
namespace auto_ring_rejection {
ElevatorAutoRingRejectorBuilder* ElevatorAutoRingRejectorBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return this;
}

ElevatorAutoRingRejectorBuilder* ElevatorAutoRingRejectorBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

ElevatorAutoRingRejectorBuilder* ElevatorAutoRingRejectorBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

std::unique_ptr<ElevatorAutoRingRejector>
ElevatorAutoRingRejectorBuilder::build() {
  std::unique_ptr<ElevatorAutoRingRejector> auto_ring_rejector{
      std::make_unique<ElevatorAutoRingRejector>()};

  auto_ring_rejector->setDelayer(m_delayer);
  auto_ring_rejector->setMutex(m_mutex);
  auto_ring_rejector->setTask(m_task);

  return auto_ring_rejector;
}
}  // namespace auto_ring_rejection
}  // namespace processes
}  // namespace driftless