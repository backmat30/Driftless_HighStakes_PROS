#ifndef __ARM_SUBSYSTEM_HPP__
#define __ARM_SUBSYSTEM_HPP__

#include <memory>

#include "pvegas/robot/subsystems/ASubsystem.hpp"
#include "pvegas/robot/subsystems/arm/IArmMotion.hpp"
#include "pvegas/robot/subsystems/arm/IRingSensor.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace arm {
class ArmSubsystem : public ASubsystem {
 private:
  // the name of the subsystem
  static constexpr char SUBSYSTEM_NAME[]{"ARM"};

  // COMMAND NAMES

  // command to go to the neutral position
  static constexpr char GO_NEUTRAL_COMMAND_NAME[]{"GO NEUTRAL"};

  // command to go to the load position
  static constexpr char GO_LOAD_COMMAND_NAME[]{"GO LOAD"};

  // command to go to the score position
  static constexpr char GO_SCORE_COMMAND_NAME[]{"GO SCORE"};

  // STATE NAMES

  // state determining if the arm is at neutral position
  static constexpr char IS_NEUTRAL_STATE_NAME[]{"IS NEUTRAL"};

  // state determining if the arm is at load position
  static constexpr char IS_LOAD_STATE_NAME[]{"IS LOAD"};

  // state determining if the arm is at score position
  static constexpr char IS_SCORE_STATE_NAME[]{"IS SCORE"};

  // state determining if there is a ring in the loading zone
  static constexpr char HAS_RING_STATE_NAME[]{"HAS RING"};

  // state determining the color of the ring in the loading zone, if applicable
  static constexpr char GET_HUE_STATE_NAME[]{"GET HUE"};

  // the arm motion controller
  std::unique_ptr<IArmMotion> m_arm_motion{};

  // the ring sensor
  std::unique_ptr<IRingSensor> m_ring_sensor{};

 public:
  // constructs a new arm subsystem
  ArmSubsystem(std::unique_ptr<IArmMotion>& arm_motion,
               std::unique_ptr<IRingSensor>& ring_sensor);

  // initialize the subsystem
  void init() override;

  // run the subsystem
  void run() override;

  // send a command to the subsystem
  void command(std::string command_name, va_list& args) override;

  // get a state of the subsystem
  void* state(std::string state_name) override;
};
}  // namespace arm
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif