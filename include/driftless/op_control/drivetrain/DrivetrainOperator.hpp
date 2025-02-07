#ifndef __DRIVETRAIN_OPERATOR_HPP__
#define __DRIVETRAIN_OPERATOR_HPP__
#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/drivetrain/EDrivetrainControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for drive train control
/// @author Matthew Backman
namespace drivetrain {

/// @brief Class to represent drive train control
/// @author Matthew Backman
class DrivetrainOperator {
 private:
  static constexpr double VOLTAGE_CONVERSION{12.0 / 127.0};

  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<robot::Robot> m_robot{};

  /// @brief Updates the voltage passed to the drive train
  /// @param left_voltage __double__ voltage passed to left motors
  /// @param right_voltage __double__ voltage passed to right motors
  void updateDriveVoltage(double left_voltage, double right_voltage);

  /// @brief Updates the robot using tank drive
  void updateTank();

  /// @brief Updates the drive train using arcade drive
  /// @param linear __EControllerAnalog__ The analog input for linear movement
  /// @param turn __EControllerAnalog__ The analog input for turning
  void updateArcade(EControllerAnalog linear, EControllerAnalog turn);

 public:
  /// @brief Constructs a new DrivetrainOperator object
  /// @param controller __std::shared_ptr<io::IController>&__ The controller
  /// used by the robot
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  DrivetrainOperator(const std::shared_ptr<io::IController> &controller,
                     const std::shared_ptr<robot::Robot> &robot);

  /// @brief Sets the drive voltage
  /// @param profile __std::unique_ptr<profiles::IProfile>&__ The profile used
  /// for control mappings
  void setDriveVoltage(std::unique_ptr<driftless::profiles::IProfile> &profile);
};

}  // namespace drivetrain
}  // namespace op_control
}  // namespace driftless
#endif