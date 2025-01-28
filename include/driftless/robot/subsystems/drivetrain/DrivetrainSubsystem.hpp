#ifndef __DRIVETRAIN_SUBSYSTEM_HPP__
#define __DRIVETRAIN_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/drivetrain/IDriveTrain.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace drivetrain {
class DrivetrainSubsystem : public ASubsystem {
 private:
  std::unique_ptr<IDrivetrain> m_drivetrain{};

 public:
  DrivetrainSubsystem(std::unique_ptr<IDrivetrain> &drivetrain);

  void init() override;

  void run() override;

  void command(ESubsystemCommand command_name, va_list &args) override;

  void *state(ESubsystemState state_name) override;
};
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif