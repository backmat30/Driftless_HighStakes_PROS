#ifndef __DEFAULT_PROFILE_HPP__
#define __DEFAULT_PROFILE_HPP__

#include <map>
#include <string>

#include "pvegas/op_control/EControl.hpp"
#include "pvegas/op_control/EControlType.hpp"
#include "pvegas/op_control/EControllerAnalog.hpp"
#include "pvegas/op_control/EControllerDigital.hpp"
#include "pvegas/op_control/arm/EArmControlMode.hpp"
#include "pvegas/op_control/clamp/EClampControlMode.hpp"
#include "pvegas/op_control/drivetrain/EDrivetrainControlMode.hpp"
#include "pvegas/op_control/elevator/EElevatorControlMode.hpp"
#include "pvegas/op_control/intake/EIntakeControlMode.hpp"
#include "pvegas/profiles/IProfile.hpp"

namespace pvegas {
namespace profiles {
class DefaultProfile : public profiles::IProfile {
 private:
  // name of profile
  static constexpr char PROFILE_NAME[]{"DEFAULT"};

  // list of subsystems to be controlled and the type of control used
  std::map<op_control::EControlType, int> CONTROL_MODE_MAP{
      {op_control::EControlType::ARM,
       static_cast<int>(op_control::arm::EArmControlMode::SINGLE_TOGGLE)},
      {op_control::EControlType::CLAMP,
       static_cast<int>(op_control::clamp::EClampControlMode::SINGLE_TOGGLE)},
      {op_control::EControlType::DRIVE,
       static_cast<int>(op_control::drivetrain::EDrivetrainControlMode::TANK)},
      {op_control::EControlType::ELEVATOR,
       static_cast<int>(op_control::elevator::EElevatorControlMode::HOLD)},
      {op_control::EControlType::INTAKE,
       static_cast<int>(
           op_control::intake::EIntakeControlMode::SINGLE_TOGGLE)}};

  // maps subsystem controls to analog inputs
  const std::map<op_control::EControl, op_control::EControllerAnalog>
      ANALOG_CONTROL_MAP{};

  // maps subsystem controls to digital inputs
  const std::map<op_control::EControl, op_control::EControllerDigital>
      DIGITAL_CONTROL_MAP{
          {op_control::EControl::ARM_TOGGLE,
           op_control::EControllerDigital::BUTTON_A},
          {op_control::EControl::ELEVATOR_SPIN,
           op_control::EControllerDigital::TRIGGER_RIGHT_BOTTOM},
          {op_control::EControl::INTAKE_TOGGLE_HEIGHT,
           op_control::EControllerDigital::TRIGGER_LEFT_BOTTOM},
          {op_control::EControl::INTAKE_SPIN,
           op_control::EControllerDigital::TRIGGER_RIGHT_BOTTOM},
          {op_control::EControl::CLAMP_TOGGLE,
           op_control::EControllerDigital::TRIGGER_RIGHT_TOP}};

 public:
  // returns profile name
  std::string getName() override;

  // returns the control mode for the given subsystem
  int getControlMode(op_control::EControlType control_type) const override;

  // sets the control mode for the given subsystem
  void setControlMode(op_control::EControlType control_type,
                      int control_mode) override;

  // gets the analog input type used for an action
  op_control::EControllerAnalog getAnalogControlMapping(
      op_control::EControl control) const override;

  // gets the digital input type used for an action
  op_control::EControllerDigital getDigitalControlMapping(
      op_control::EControl control) const override;
};
}  // namespace profiles
}  // namespace pvegas
#endif