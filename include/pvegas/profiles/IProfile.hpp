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
//default destructor
  virtual ~IProfile() = 0;

//returns the name of the profile
  virtual std::string getName() = 0;

//gets the control mode for a given subsystem
  virtual int getControlMode(op_control::EControlType control_type) const = 0;

//sets the control mode for a given subsystem
  virtual void setControlMode(op_control::EControlType control_type,
                              int control_mode) = 0;

//gets the analog input for a given action
  virtual op_control::EControllerAnalog
  getAnalogControlMapping(op_control::EControl control) const = 0;

//gets the digital input for a given action
  virtual op_control::EControllerDigital
  getDigitalControlMapping(op_control::EControl control) const = 0;
};
} // namespace profile
} // namespace pvegas
#endif