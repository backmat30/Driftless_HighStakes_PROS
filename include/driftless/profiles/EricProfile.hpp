#ifndef __ERIC_PROFILE_HPP__
#define __ERIC_PROFILE_HPP__

#include <map>
#include <string>

#include "driftless/op_control/drivetrain/EDrivetrainControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"

/// @brief Namespace for driftless library code
namespace driftless {
/// @brief Namespace for user control profiles
namespace profiles {
/// @brief User control profile for Eric
class EricProfile : public driftless::profiles::IProfile {
 private:
  /// @brief Name of the profile
  static constexpr char PROFILE_NAME[]{"ERIC"};

  /// @brief List of subsystems to be controlled and the type of control used
  std::map<op_control::EControlType, int> CONTROL_MODE_MAP{
      {op_control::EControlType::DRIVE,
       static_cast<int>(
           op_control::drivetrain::EDrivetrainControlMode::ARCADE)}};

  /// @brief Maps subsystem controls to analog inputs
  const std::map<op_control::EControl, op_control::EControllerAnalog>
      ANALOG_CONTROL_MAP{{op_control::EControl::DRIVE_ARCADE_LINEAR,
                          op_control::EControllerAnalog::JOYSTICK_RIGHT_Y},
                         {op_control::EControl::DRIVE_ARCADE_TURN,
                          op_control::EControllerAnalog::JOYSTICK_LEFT_X}};

  /// @brief Maps subsystem controls to digital inputs
  const std::map<op_control::EControl, op_control::EControllerDigital>
      DIGITAL_CONTROL_MAP{};

 public:
  /// @brief Gets the name of the profile
  /// @return The name as a string
  std::string getName() override;

  /// @brief Gets the control mode of the given control type
  /// @param control_type The control type
  /// @return The control mode as an integer
  int getControlMode(op_control::EControlType control_type) const override;

  /// @brief Sets the control mode of the given control type
  /// @param control_type The control type
  /// @param control_mode The new control mode
  void setControlMode(op_control::EControlType control_type, int control_mode) override;

  /// @brief Gets the analog control mapped to the given control
  /// @param control The control
  /// @return The analog control mapped to the control
  op_control::EControllerAnalog getAnalogControlMapping(op_control::EControl control) const override;

  /// @brief Gets the digital control mapped to the given control
  /// @param control The control
  /// @return The digital control mapped to the control
  op_control::EControllerDigital getDigitalControlMapping(op_control::EControl control) const override;
};
}  // namespace profiles
}  // namespace driftless
#endif