#ifndef __I_PROFILE_HPP__
#define __I_PROFILE_HPP__

#include <string>

#include "driftless/op_control/EControl.hpp"
#include "driftless/op_control/EControlType.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/EStartupConfig.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for user control profiles
/// @author Matthew Backman
namespace profiles {

/// @brief Interface for user control profiles
/// @author Matthew Backman
class IProfile {
 public:
  /// @brief Default destructor
  virtual ~IProfile() = default;

  /// @brief Gets the name of the profile
  /// @return __std::string__ The name as a string
  virtual std::string getName() = 0;

  /// @brief Gets the control mode for a given subsystem
  /// @param control_type __op_control::EControlType__ The control type
  /// @return __int__ The control mode as an integer
  virtual int getControlMode(op_control::EControlType control_type) const = 0;

  /// @brief Sets the control mode for a given subsystem
  /// @param control_type __op_control::EControlType__ The control type
  /// @param control_mode __int__ The new control mode
  virtual void setControlMode(op_control::EControlType control_type,
                              int control_mode) = 0;

  /// @brief Gets the analog input for a given action
  /// @param control __op_control::EControl__ The control
  /// @return __op_control::EControllerAnalog__ The analog input type
  virtual op_control::EControllerAnalog getAnalogControlMapping(
      op_control::EControl control) const = 0;

  /// @brief Gets the digital input for a given action
  /// @param control __op_control::EControl__ The control
  /// @return __op_control::EControllerDigital__ The digital input type
  virtual op_control::EControllerDigital getDigitalControlMapping(
      op_control::EControl control) const = 0;

  /// @brief Gets the value of the desired startup configuration
  /// @param startup_config __op_control::EStartupConfig__ The desired startup
  /// configuration
  /// @return __bool__ The value of the desired startup configuration
  virtual bool getStartupConfig(
      op_control::EStartupConfig startup_config) const = 0;
};
}  // namespace profiles
}  // namespace driftless
#endif