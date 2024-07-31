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
  static constexpr char PROFILE_NAME[]{"DEFAULT"};

  std::map<op_control::EControlType, int> CONTROL_MODE_MAP{
      {op_control::EControlType::DRIVE,
       static_cast<int>(op_control::drivetrain::EDrivetrainControlMode::TANK)}};

  const std::map<op_control::EControl, op_control::EControllerAnalog>
      ANALOG_CONTROL_MAP{};

  const std::map<op_control::EControl, op_control::EControllerDigital>
      DIGITAL_CONTROL_MAP{};

public:
  std::string getName() override;

  int getControlMode(op_control::EControlType control_type) const override;

  void setControlMode(op_control::EControlType control_type,
                      int control_mode) override;

  op_control::EControllerAnalog
  getAnalogControlMapping(op_control::EControl control) const override;

  op_control::EControllerDigital
  getDigitalControlMapping(op_control::EControl control) const override;
};
} // namespace profiles
} // namespace pvegas
#endif