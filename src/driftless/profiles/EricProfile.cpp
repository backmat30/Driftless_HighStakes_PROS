#include "driftless/profiles/EricProfile.hpp"

namespace driftless {
namespace profiles {
std::string EricProfile::getName() { return PROFILE_NAME; }

int EricProfile::getControlMode(op_control::EControlType control_type) const {
  int mode{};
  if (CONTROL_MODE_MAP.contains(control_type)) {
    mode = CONTROL_MODE_MAP.at(control_type);
  }
  return mode;
}

void EricProfile::setControlMode(op_control::EControlType control_type,
                                 int control_mode) {
  CONTROL_MODE_MAP[control_type] = control_mode;
}

op_control::EControllerAnalog EricProfile::getAnalogControlMapping(
    op_control::EControl control) const {
  op_control::EControllerAnalog analogMapping{
      op_control::EControllerAnalog::NONE};
  if (ANALOG_CONTROL_MAP.contains(control)) {
    analogMapping = ANALOG_CONTROL_MAP.at(control);
  }
  return analogMapping;
}

op_control::EControllerDigital EricProfile::getDigitalControlMapping(
    op_control::EControl control) const {
  op_control::EControllerDigital digitalMapping{
      op_control::EControllerDigital::NONE};
  if (DIGITAL_CONTROL_MAP.contains(control)) {
    digitalMapping = DIGITAL_CONTROL_MAP.at(control);
  }
  return digitalMapping;
}

bool EricProfile::getStartupConfig(
    op_control::EStartupConfig startup_config) const {
  bool config{};
  if (STARTUP_CONFIG_MAP.contains(startup_config)) {
    config = STARTUP_CONFIG_MAP.at(startup_config);
  }
  return config;
}
}  // namespace profiles
}  // namespace driftless