#ifndef __CONTROLLER_SWAP_OPERATOR_HPP__
#define __CONTROLLER_SWAP_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/profiles/IProfile.hpp"

namespace driftless {
namespace op_control {
namespace controller_swap {
class ControllerSwapOperator {
 private:
  std::shared_ptr<io::IController> m_controller{};

 public:
  ControllerSwapOperator(const std::shared_ptr<io::IController>& controller);

  void update(const std::unique_ptr<profiles::IProfile>& profile);
};
}  // namespace controller_swap
}  // namespace op_control
}  // namespace driftless
#endif