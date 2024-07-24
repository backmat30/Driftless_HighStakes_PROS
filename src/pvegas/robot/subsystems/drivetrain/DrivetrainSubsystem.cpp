#include "pvegas/robot/subsystems/drivetrain/DrivetrainSubsystem.hpp"
#include "pvegas/robot/subsystems/ASubsystem.hpp"
#include "pvegas/robot/subsystems/drivetrain/Velocity.hpp"
#include <cstdarg>

namespace pvegas {
namespace robot {
namespace subsystems {
namespace drivetrain {
DrivetrainSubsystem::DrivetrainSubsystem(
    std::unique_ptr<IDrivetrain> &drivetrain)
    : ASubsystem{SUBSYSTEM}, m_drivetrain(std::move(drivetrain)) {}

void DrivetrainSubsystem::init() { m_drivetrain->init(); }

void DrivetrainSubsystem::run() { m_drivetrain->run(); }

void DrivetrainSubsystem::command(std::string command_name, va_list &args) {
  if (command_name == SET_VELOCITY_COMMAND) {
    double left_velocity{va_arg(args, double)};
    double right_velocity{va_arg(args, double)};
    Velocity velocity{left_velocity, right_velocity};
    m_drivetrain->setVelocity(velocity);
  } else if (command_name == SET_VOLTAGE_COMMAND) {
    double left_voltage{va_arg(args, double)};
    double right_voltage{va_arg(args, double)};
    m_drivetrain->setVoltage(left_voltage, right_voltage);
  }
}

void* DrivetrainSubsystem::state(std::string state_name){
    void* result{nullptr};

    if(state_name == GET_VELOCITY){
        Velocity* velocity{new Velocity{m_drivetrain->getVelocity()}};
        result = velocity;
    } else if(state_name == GET_RADIUS){
        double* radius{new double{m_drivetrain->getDriveRadius()}};
        result = radius;
    }
    return result;
}
} // namespace drivetrain
} // namespace subsystems
} // namespace robot
} // namespace pvegas