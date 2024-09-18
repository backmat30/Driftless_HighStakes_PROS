#include "pvegas/op_control/drivetrain/DrivetrainOperator.hpp"
#include "pvegas/io/IController.hpp"
#include "pvegas/robot/Robot.hpp"
#include <memory>
namespace pvegas {
namespace op_control {
namespace drivetrain {
void DrivetrainOperator::updateDriveVoltage(double left_voltage,
                                            double right_voltage) {
  if (m_robot) {
    m_robot->sendCommand(DRIVE_SUBSYSTEM_NAME, SET_VOLTAGE_COMMAND,
                         left_voltage, right_voltage);
  }
}

void DrivetrainOperator::updateTank() {
  double left_voltage{
      m_controller->getAnalog(EControllerAnalog::JOYSTICK_LEFT_Y) *
      VOLTAGE_CONVERSION};
  double right_voltage{
      m_controller->getAnalog(EControllerAnalog::JOYSTICK_RIGHT_Y) *
      VOLTAGE_CONVERSION};
}

DrivetrainOperator::DrivetrainOperator(
    const std::shared_ptr<io::IController> &controller,
    const std::shared_ptr<robot::Robot> &robot)
    : m_controller{controller}, m_robot{robot} {}

void DrivetrainOperator::setDriveVoltage() {
  if (!m_controller) {
    updateDriveVoltage(0, 0);
    return;
  }
  updateTank();
}
} // namespace drivetrain
} // namespace op_control
} // namespace pvegas