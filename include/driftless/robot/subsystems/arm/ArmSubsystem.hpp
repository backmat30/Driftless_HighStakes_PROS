#ifndef __ARM_SUBSYSTEM_HPP__
#define __ARM_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/arm/IArmMotion.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace arm {
class ArmSubsystem : public ASubsystem {
 private:
  // the name of the subsystem
  static constexpr char SUBSYSTEM_NAME[]{"ARM"};

  // COMMAND NAMES

  // command to calibrate the arm
  static constexpr char CALIBRATE_COMMAND_NAME[]{"CALIBRATE"};

  // command to go to the neutral position
  static constexpr char GO_NEUTRAL_COMMAND_NAME[]{"GO NEUTRAL"};

  // command to go to the load position
  static constexpr char GO_LOAD_COMMAND_NAME[]{"GO LOAD"};

  // command to go to the ready position
  static constexpr char GO_READY_COMMAND_NAME[]{"GO READY"};

  // command to go to the score position
  static constexpr char GO_SCORE_COMMAND_NAME[]{"GO SCORE"};

  // command to go to the rush position
  static constexpr char GO_RUSH_COMMAND_NAME[]{"GO RUSH"};

  // command to go to the alliance stake position
  static constexpr char GO_ALLIANCE_STAKE_COMMAND_NAME[]{"GO ALLIANCE STAKE"};

  // command to go to the previous position
  static constexpr char GO_PREVIOUS_COMMAND_NAME[]{"GO PREVIOUS"};

  // STATE NAMES

  // state determining if the arm is at neutral position
  static constexpr char IS_NEUTRAL_STATE_NAME[]{"IS NEUTRAL"};

  // state determining if the arm is moving to the neutral position
  static constexpr char IS_GOING_NEUTRAL_STATE_NAME[]{"IS GOING NEUTRAL"};

  // state determining if the arm is at load position
  static constexpr char IS_LOAD_STATE_NAME[]{"IS LOAD"};

  // state determining if the arm is going to the load position
  static constexpr char IS_GOING_LOAD_STATE_NAME[]{"IS GOING LOAD"};

  // state determining if the arm is at the ready position
  static constexpr char IS_READY_STATE_NAME[]{"IS READY"};

  // state determining if the arm is going to the ready position
  static constexpr char IS_GOING_READY_STATE_NAME[]{"IS GOING READY"};

  // state determining if the arm is at score position
  static constexpr char IS_SCORE_STATE_NAME[]{"IS SCORE"};

  // state determining if the arm is going to the score position
  static constexpr char IS_GOING_SCORE_STATE_NAME[]{"IS GOING SCORE"};

  // state determining if the arm is at the rush position
  static constexpr char IS_RUSH_STATE_NAME[]{"IS RUSH"};

  // state determining if the arm is going to the rush position
  static constexpr char IS_GOING_RUSH_STATE_NAME[]{"IS GOING RUSH"};

  // state determining if the arm is at the alliance stake position
  static constexpr char IS_ALLIANCE_STAKE_STATE_NAME[]{"IS ALLIANCE STAKE"};

  // state determining if the arm is going to the alliance stake position
  static constexpr char IS_GOING_ALLIANCE_STAKE_STATE_NAME[]{
      "IS GOING ALLIANCE STAKE"};

  // the arm motion controller
  std::unique_ptr<IArmMotion> m_arm_motion{};

 public:
  // constructs a new arm subsystem
  ArmSubsystem(std::unique_ptr<IArmMotion>& arm_motion);

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
}  // namespace driftless
#endif