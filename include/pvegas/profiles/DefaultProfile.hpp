#ifndef __DEFAULT_PROFILE_HPP__
#define __DEFAULT_PROFILE_HPP__

#include <map>
#include <string>

#include "pvegas/op_control/EControl.hpp"
#include "pvegas/op_control/EControlType.hpp"
#include "pvegas/op_control/EControllerAnalog.hpp"
#include "pvegas/op_control/EControllerDigital.hpp"
#include "pvegas/op_control/drivetrain/EDrivetrainControlMode.hpp"
#include "pvegas/profiles/IProfile.hpp"

namespace pvegas {
namespace profiles {
class DefaultProfile : public profile::IProfile {
private:
//name of profile
  static constexpr char PROFILE_NAME[]{"DEFAULT"};

//list of subsystems to be controlled and the type of control used
  std::map<op_control::EControlType, int> CONTROL_MODE_MAP{
      {op_control::EControlType::DRIVE,
       static_cast<int>(op_control::drivetrain::EDrivetrainControlMode::TANK)}};

//maps subsystem controls to analog inputs
  const std::map<op_control::EControl, op_control::EControllerAnalog>
      ANALOG_CONTROL_MAP{};

//maps subsystem controls to digital inputs
  const std::map<op_control::EControl, op_control::EControllerDigital>
      DIGITAL_CONTROL_MAP{};

public:
//returns profile name
  std::string getName() override;

//returns the control mode for the given subsystem
  int getControlMode(op_control::EControlType control_type) const override;

//sets the control mode for the given subsystem
  void setControlMode(op_control::EControlType control_type,
                      int control_mode) override;

//gets the analog input type used for an action
  op_control::EControllerAnalog
  getAnalogControlMapping(op_control::EControl control) const override;

//gets the digital input type used for an action
  op_control::EControllerDigital
  getDigitalControlMapping(op_control::EControl control) const override;
};
} // namespace profiles
} // namespace pvegas
#endif