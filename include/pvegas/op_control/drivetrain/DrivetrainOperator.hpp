#ifndef __DRIVETRAIN_OPERATOR_HPP__
#define __DRIVETRAIN_OPERATOR_HPP__
#include "pvegas/pros_adapters/ProsController.hpp"
#include "pvegas/op_control/EControllerAnalog.hpp"
#include "pvegas/op_control/EControllerDigital.hpp"
#include "pvegas/robot/Robot.hpp"
#include <memory>

namespace pvegas {
namespace op_control {
namespace drivetrain {
class DrivetrainOperator {
private:
  static constexpr char DRIVE_SUBSYSTEM_NAME[]{"DIFFERENTIAL DRIVE"};

  static constexpr char SET_VOLTAGE_COMMAND[]{"SET VOLTAGE"};

  static constexpr double VOLTAGE_CONVERSION{12.0};

  std::shared_ptr<pros_adapters::ProsController>
      m_controller{};

  std::shared_ptr<robot::Robot> m_robot{};

  void updateDriveVoltage(double left_voltage, double right_voltage);

  void updateTank();

public:
  DrivetrainOperator(
      const std::shared_ptr<pros_adapters::ProsController>
          &controller,
      const std::shared_ptr<robot::Robot> &robot);

  void setDriveVoltage();
};
} // namespace drivetrain
} // namespace op_control
} // namespace pvegas
#endif