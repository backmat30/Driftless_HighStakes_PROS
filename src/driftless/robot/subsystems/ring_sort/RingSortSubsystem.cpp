#include "driftless/robot/subsystems/ring_sort/RingSortSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace ring_sort {
RingSortSubsystem::RingSortSubsystem(std::unique_ptr<IRingSort>& ring_sort)
    : ASubsystem{ESubsystem::RING_SORT}, m_ring_sort{std::move(ring_sort)} {}

void RingSortSubsystem::init() { m_ring_sort->init(); }

void RingSortSubsystem::run() { m_ring_sort->run(); }

void RingSortSubsystem::command(ESubsystemCommand command_name, va_list& args) {
}

void* RingSortSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};

  if (state_name == ESubsystemState::RING_SORT_HAS_RING) {
    result = new bool{m_ring_sort->hasRing()};
  } else if (state_name == ESubsystemState::RING_SORT_GET_HUE) {
    result = new double{m_ring_sort->getRingHue()};
  } else if (state_name == ESubsystemState::RING_SORT_GET_DISTANCE_TO_END) {
    result = new double{m_ring_sort->getDistanceToElevatorEnd()};
  } else if (state_name == ESubsystemState::RING_SORT_GET_RGB) {
    result = new io::RGBValue{m_ring_sort->getRingRGB()};
  }

  return result;
}
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless