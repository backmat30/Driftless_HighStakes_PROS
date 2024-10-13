#ifndef __DRIVETRAIN_OPERATOR_HPP__
#define __DRIVETRAIN_OPERATOR_HPP__
#include <memory>

#include "pvegas/io/IController.hpp"
#include "pvegas/op_control/EControllerAnalog.hpp"
#include "pvegas/op_control/EControllerDigital.hpp"
#include "pvegas/robot/Robot.hpp"

namespace pvegas {
namespace op_control {
namespace drivetrain {
class DrivetrainOperator {
 private:
  static constexpr char DRIVE_SUBSYSTEM_NAME[]{"DIFFERENTIAL DRIVE"};

  static constexpr char SET_VOLTAGE_COMMAND[]{"SET VOLTAGE"};

  static constexpr double VOLTAGE_CONVERSION{12.0};

  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<robot::Robot> m_robot{};

  void updateDriveVoltage(double left_voltage, double right_voltage);

  void updateTank();

 public:
  DrivetrainOperator(const std::shared_ptr<io::IController> &controller,
                     const std::shared_ptr<robot::Robot> &robot);

  void setDriveVoltage();
};
}  // namespace drivetrain
}  // namespace op_control
}  // namespace pvegas
#endif