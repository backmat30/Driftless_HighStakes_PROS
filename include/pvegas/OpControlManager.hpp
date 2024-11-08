#ifndef __OP_CONTROL_MANAGER_HPP__
#define __OP_CONTROL_MANAGER_HPP__

#include <cstdint>
#include <memory>

#include "pvegas/alliance/Alliance.hpp"
#include "pvegas/control/ControlSystem.hpp"
#include "pvegas/io/IController.hpp"
#include "pvegas/op_control/arm/ArmOperator.hpp"
#include "pvegas/op_control/clamp/ClampOperator.hpp"
#include "pvegas/op_control/drivetrain/DrivetrainOperator.hpp"
#include "pvegas/op_control/elevator/ElevatorOperator.hpp"
#include "pvegas/op_control/intake/IntakeOperator.hpp"
#include "pvegas/profiles/IProfile.hpp"
#include "pvegas/robot/Robot.hpp"
#include "pvegas/rtos/IClock.hpp"
#include "pvegas/rtos/IDelayer.hpp"

namespace driftless {
class OpControlManager {
 private:
  // loop delay
  static constexpr uint32_t CONTROL_DELAY{10};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<profiles::IProfile> m_profile{};

  // the current alliance
  driftless::alliance::Alliance m_alliance{};

 public:
  // constructor
  OpControlManager(const std::shared_ptr<rtos::IClock> &clock,
                   const std::unique_ptr<rtos::IDelayer> &delayer);

  // defines the controller profile to use
  void setProfile(std::unique_ptr<profiles::IProfile> &profile);

  // defines the alliance the robot is on
  void setAlliance(driftless::alliance::Alliance alliance);

  // initializes op control
  void init(std::shared_ptr<control::ControlSystem> control_system,
            std::shared_ptr<io::IController> controller,
            std::shared_ptr<robot::Robot> robot);

  // runs op control
  void run(std::shared_ptr<control::ControlSystem> control_system,
           std::shared_ptr<io::IController> controller,
           std::shared_ptr<robot::Robot> robot);
};
}  // namespace pvegas
#endif