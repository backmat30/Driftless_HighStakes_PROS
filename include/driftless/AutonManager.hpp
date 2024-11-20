#ifndef __AUTON_MANAGER_HPP__
#define __AUTON_MANAGER_HPP__

#include <memory>

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/auton/IAuton.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"

namespace driftless {
class AutonManager {
 private:
  std::shared_ptr<alliance::IAlliance> m_alliance{};

  std::unique_ptr<auton::IAuton> m_auton{};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

 public:
  AutonManager(const std::shared_ptr<rtos::IClock>& clock,
               const std::unique_ptr<rtos::IDelayer>& delayer);

  void setAlliance(const std::shared_ptr<alliance::IAlliance>& alliance);

  void setAuton(std::unique_ptr<auton::IAuton>& auton);

  void initAuton(std::shared_ptr<robot::Robot>& robot,
                 std::shared_ptr<control::ControlSystem>& control_system);

  void runAuton(
      std::shared_ptr<driftless::robot::Robot>& robot,
      std::shared_ptr<driftless::control::ControlSystem>& control_system);
};
}  // namespace driftless
#endif