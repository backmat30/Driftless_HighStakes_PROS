#include "driftless/op_control/color_sort/ColorSortOperator.hpp"

namespace driftless {
namespace op_control {
namespace color_sort {
ColorSortOperator::ColorSortOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<processes::ProcessSystem>& process_system)
    : m_controller{controller}, m_process_system{process_system} {}

void ColorSortOperator::updateRingRejection(
    const std::unique_ptr<profiles::IProfile>& profile,
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  EControllerDigital toggle_ring_rejection{
      profile->getDigitalControlMapping(EControl::COLOR_SORT_TOGGLE)};

  bool do_toggle_ring_rejection{
      m_controller->getNewDigital(toggle_ring_rejection)};

  bool is_ring_rejection_paused{*static_cast<bool*>(m_process_system->getState(
      processes::EProcess::AUTO_RING_REJECTION,
      processes::EProcessState::AUTO_RING_REJECTION_IS_PAUSED))};

  if (do_toggle_ring_rejection) {
    if (is_ring_rejection_paused) {
      m_process_system->resume(processes::EProcess::AUTO_RING_REJECTION);
    } else {
      m_process_system->pause(processes::EProcess::AUTO_RING_REJECTION);
    }
  }
}
}  // namespace color_sort
}  // namespace op_control
}  // namespace driftless