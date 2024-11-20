#include "driftless/robot/subsystems/ring_sort/RingSortSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace ring_sort {
RingSortSubsystem::RingSortSubsystem(std::unique_ptr<IRingSort>& ring_sort)
    : ASubsystem{RING_SORT_SUBSYSTEM_NAME}, m_ring_sort{std::move(ring_sort)} {}

void RingSortSubsystem::init() { m_ring_sort->init(); }

void RingSortSubsystem::run() { m_ring_sort->run(); }

void RingSortSubsystem::command(std::string command_name, va_list& args) {}

void* RingSortSubsystem::state(std::string state_name) {
  void* result{nullptr};

  if (state_name == HAS_RING_STATE_NAME) {
    result = new bool{m_ring_sort->hasRing()};
  } else if (state_name == GET_HUE_STATE_NAME) {
    result = new double{m_ring_sort->getRingHue()};
  } else if (state_name == GET_DISTANCE_TO_END_STATE_NAME) {
    result = new double{m_ring_sort->getDistanceToElevatorEnd()};
  } else if (state_name == GET_RGB_STATE_NAME) {
    result = new io::RGBValue{m_ring_sort->getRingRGB()};
  }

  return result;
}
}  // namespace ring_sort
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless