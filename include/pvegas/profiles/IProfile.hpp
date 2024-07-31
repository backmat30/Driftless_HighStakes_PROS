#ifndef __I_PROFILE_HPP__
#define __I_PROFILE_HPP__

#include <string>

#include "pvegas/op_control/EControl.hpp"
#include "pvegas/op_control/EControlType.hpp"
#include "pvegas/op_control/EControllerAnalog.hpp"
#include "pvegas/op_control/EControllerDigital.hpp"


namespace pvegas {
namespace profile {
class IProfile {
public:
  virtual ~IProfile() = 0;

  virtual std::string getName() = 0;

  virtual int getControlMode(op_control::EControlType control_type) const = 0;

  virtual void setControlMode(op_control::EControlType control_type,
                              int control_mode) = 0;

  virtual op_control::EControllerAnalog
  getAnalogControlMapping(op_control::EControl control) const = 0;

  virtual op_control::EControllerDigital
  getDigitalControlMapping(op_control::EControl control) const = 0;
};
} // namespace profile
} // namespace pvegas
#endif