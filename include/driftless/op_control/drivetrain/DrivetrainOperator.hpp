#ifndef __DRIVETRAIN_OPERATOR_HPP__
#define __DRIVETRAIN_OPERATOR_HPP__
#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/drivetrain/EDrivetrainControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"

namespace driftless {
namespace op_control {
namespace drivetrain {
class DrivetrainOperator {
 private:
  static constexpr char DRIVE_SUBSYSTEM_NAME[]{"DIFFERENTIAL DRIVE"};

  static constexpr char SET_VOLTAGE_COMMAND[]{"SET VOLTAGE"};

  static constexpr double VOLTAGE_CONVERSION{12.0 / 127.0};

  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<robot::Robot> m_robot{};

  void updateDriveVoltage(double left_voltage, double right_voltage);

  void updateTank();

  ///@brief updates the drive train using arcade drive
  void updateArcade(EControllerAnalog linear, EControllerAnalog turn);

 public:
  DrivetrainOperator(const std::shared_ptr<io::IController> &controller,
                     const std::shared_ptr<robot::Robot> &robot);

  void setDriveVoltage(std::unique_ptr<driftless::profiles::IProfile> &profile);
};
}  // namespace drivetrain
}  // namespace op_control
}  // namespace driftless
#endif