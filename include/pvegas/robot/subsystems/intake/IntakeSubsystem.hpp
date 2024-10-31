#ifndef __INTAKE_SUBSYSTEM_HPP__
#define __INTAKE_SUBSYSTEM_HPP__

#include <memory>

#include "pvegas/robot/subsystems/ASubsystem.hpp"
#include "pvegas/robot/subsystems/intake/IHeightControl.hpp"
#include "pvegas/robot/subsystems/intake/IIntake.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace intake {
class IntakeSubsystem : public ASubsystem {
 private:
  // name of the subsystem
  static constexpr char SUBSYSTEM_NAME[]{"INTAKE"};

  // COMMAND NAMES

  // command to spin the intake wheels
  static constexpr char SPIN_INTAKE_COMMAND_NAME[]{"SPIN"};

  // command to set the height of the intake
  static constexpr char SET_HEIGHT_COMMAND_NAME[]{"SET HEIGHT"};

  // STATE NAMES

  // gets the position of the intake
  static constexpr char GET_HEIGHT_STATE_NAME[]{"GET HEIGHT"};

  // the intake object
  std::unique_ptr<IIntake> m_intake{};

  // the intake height controller
  std::unique_ptr<IHeightControl> m_height_control{};

 public:
  // constructs a new intake subsystem
  IntakeSubsystem(std::unique_ptr<IIntake>& intake,
                  std::unique_ptr<IHeightControl>& height_control);

  // initialize the subsystem
  void init() override;

  // run the subsystem
  void run() override;

  // send a command to the subsystem
  void command(std::string command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(std::string state_name) override;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif