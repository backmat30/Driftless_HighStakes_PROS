#ifndef __MOTION_CONTROL_HPP__
#define __MOTION_CONTROL_HPP__

#include <memory>

#include "pvegas/control/AControl.hpp"
#include "pvegas/control/motion/EMotionType.hpp"
#include "pvegas/control/motion/IDriveStraight.hpp"
#include "pvegas/control/motion/IGoToPoint.hpp"
#include "pvegas/control/motion/ITurn.hpp"

namespace driftless {
namespace control {
namespace motion {
class MotionControl : public driftless::control::AControl {
 private:
  // the name of the control
  static constexpr char CONTROL_NAME[]{"MOTION"};

  // name of the drive straight command
  static constexpr char DRIVE_STRAIGHT_COMMAND_NAME[]{"DRIVE STRAIGHT"};

  // name of the go to point command
  static constexpr char GO_TO_POINT_COMMAND_NAME[]{"GO TO POINT"};

  // name of the turn to angle command
  static constexpr char TURN_TO_ANGLE_COMMAND_NAME[]{"TURN TO ANGLE"};

  // name of the turn to point command
  static constexpr char TURN_TO_POINT_COMMAND_NAME[]{"TURN TO POINT"};

  // name of the drive straight set velocity command
  static constexpr char SET_DRIVE_STRAIGHT_VELOCITY_COMMAND_NAME[]{
      "SET DRIVE STRAIGHT VELOCITY"};

  // name of the go to point set velocity command
  static constexpr char SET_GO_TO_POINT_VELOCITY_COMMAND_NAME[]{
      "SET GO TO POINT VELOCITY"};

  // name of the drive straight target reached state
  static constexpr char DRIVE_STRAIGHT_TARGET_REACHED_STATE_NAME[]{
      "DRIVE STRAIGHT TARGET REACHED"};

  // name of the go to point target reached state
  static constexpr char GO_TO_POINT_TARGET_REACHED_STATE_NAME[]{
      "GO TO POINT TARGET REACHED"};

  // name of the turn target reached state
  static constexpr char TURN_TARGET_REACHED_STATE_NAME[]{"TURN TARGET REACHED"};

  // drive straight algorithm
  std::unique_ptr<driftless::control::motion::IDriveStraight> m_drive_straight{};

  // go to point algorithm
  std::unique_ptr<driftless::control::motion::IGoToPoint> m_go_to_point{};

  // turn algorithm
  std::unique_ptr<driftless::control::motion::ITurn> m_turn{};

  driftless::control::motion::EMotionType m_motion_type{EMotionType::NONE};

 public:
  // constructor
  MotionControl(
      std::unique_ptr<driftless::control::motion::IDriveStraight>& drive_straight,
      std::unique_ptr<driftless::control::motion::IGoToPoint>& go_to_point,
      std::unique_ptr<driftless::control::motion::ITurn>& turn);

  // initialize the control
  void init() override;

  // run the control
  void run() override;

  // pause the control
  void pause() override;

  // resume the control
  void resume() override;

  // send a command to the control
  void command(std::string command_name, va_list& args) override;

  // get a state of the control
  void* state(std::string state_name) override;
};
}  // namespace motion
}  // namespace control
}  // namespace pvegas
#endif