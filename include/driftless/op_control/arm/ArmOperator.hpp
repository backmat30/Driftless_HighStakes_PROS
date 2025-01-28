#ifndef __ARM_OPERATOR_HPP__
#define __ARM_OPERATOR_HPP__

#include "driftless/alliance/IAlliance.hpp"
#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/arm/EArmControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

namespace driftless {
namespace op_control {
namespace arm {
class ArmOperator {
 private:
  // name of the blue alliance
  static constexpr char BLUE_ALLIANCE_NAME[]{"BLUE"};

  // name of the red alliance
  static constexpr char RED_ALLIANCE_NAME[]{"RED"};

  // the controller used
  std::shared_ptr<driftless::io::IController> m_controller{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  /// @brief determines if the robot has an alliance ring loaded
  bool hasAllianceRing(const std::shared_ptr<alliance::IAlliance>& alliance);

  /// @depricated: Use color sort process
  /// @brief determines if the robot has an opposing alliance ring loaded
  bool hasOpposingRing(const std::shared_ptr<alliance::IAlliance>& alliance);

  // updates the arm using split toggle
  void updateSplitToggle(EControllerDigital neutral, EControllerDigital load,
                         EControllerDigital ready, EControllerDigital score,
                         const std::shared_ptr<alliance::IAlliance>& alliance);

  // update the arm using single toggle
  void updateSmartToggle(EControllerDigital toggle, EControllerDigital rush,
                         EControllerDigital calibrate,
                         EControllerDigital alliance_stake,
                         const std::shared_ptr<alliance::IAlliance>& alliance);

 public:
  // constructs a new arm operator
  ArmOperator(const std::shared_ptr<driftless::io::IController>& controller,
              const std::shared_ptr<driftless::robot::Robot>& robot);

  // update the arm
  void update(const std::unique_ptr<driftless::profiles::IProfile>& profile,
              const std::shared_ptr<alliance::IAlliance>& alliance);
};
}  // namespace arm
}  // namespace op_control
}  // namespace driftless
#endif