#ifndef __DRIVETRAIN_SUBSYSTEM_HPP__
#define __DRIVETRAIN_SUBSYSTEM_HPP__

#include "pvegas/robot/subsystems/ASubsystem.hpp"
#include "pvegas/robot/subsystems/drivetrain/IDriveTrain.hpp"
#include <memory>

namespace pvegas {
namespace robot {
namespace subsystems {
namespace drivetrain {
class DrivetrainSubsystem : public ASubsystem {
private:
  static constexpr char SUBSYSTEM[]{"DIFFERENTIAL DRIVE"};
  static constexpr char SET_VELOCITY_COMMAND[]{"SET VELOCITY"};
  static constexpr char SET_VOLTAGE_COMMAND[]{"SET VOLTAGE"};
  static constexpr char GET_VELOCITY[]{"GET VELOCITY"};
  static constexpr char GET_RADIUS[]{"GET RADIUS"};

  std::unique_ptr<IDrivetrain> m_drivetrain{};

public:
  DrivetrainSubsystem(std::unique_ptr<IDrivetrain> &drivetrain);

  void init() override;

  void run() override;

  void command(std::string command_name, va_list &args) override;

  void *state(std::string state_name) override;
};
} // namespace drivetrain
} // namespace subsystems
} // namespace robot
} // namespace pvegas
#endif