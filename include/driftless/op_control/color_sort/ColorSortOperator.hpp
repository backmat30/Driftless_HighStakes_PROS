#ifndef __COLOR_SORT_OPERATOR_HPP__
#define __COLOR_SORT_OPERATOR_HPP__

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/processes/EProcess.hpp"
#include "driftless/processes/EProcessCommand.hpp"
#include "driftless/processes/EProcessState.hpp"
#include "driftless/processes/ProcessSystem.hpp"
#include "driftless/profiles/IProfile.hpp"

namespace driftless {
namespace op_control {
namespace color_sort {
class ColorSortOperator {
 private:
  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<processes::ProcessSystem> m_process_system{};

 public:
  ColorSortOperator(
      const std::shared_ptr<io::IController>& controller,
      const std::shared_ptr<processes::ProcessSystem>& process_system);

  void updateRingRejection(
      const std::unique_ptr<profiles::IProfile>& profile,
      const std::shared_ptr<alliance::IAlliance>& alliance);
};
}  // namespace color_sort
}  // namespace op_control
}  // namespace driftless
#endif