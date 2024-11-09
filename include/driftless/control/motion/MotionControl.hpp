#ifndef __MOTION_CONTROL_HPP__
#define __MOTION_CONTROL_HPP__

#include <memory>

#include "driftless/control/AControl.hpp"
#include "driftless/control/motion/EMotionType.hpp"
#include "driftless/control/motion/IDriveStraight.hpp"
#include "driftless/control/motion/IGoToPoint.hpp"
#include "driftless/control/motion/ITurn.hpp"

/// @brief Namespace for driftless library code
namespace driftless {
/// @brief Namespace for control algorithms
namespace control {
/// @brief Namespace for basic motion control algorithms
namespace motion {
/// @brief Class to hold and control all motion algorithms
class MotionControl : public driftless::control::AControl {
 private:
  /// @brief The name of the control
  static constexpr char CONTROL_NAME[]{"MOTION"};

  /// @brief The name of the command to drive straight
  static constexpr char DRIVE_STRAIGHT_COMMAND_NAME[]{"DRIVE STRAIGHT"};

  /// @brief The name of the command to go to a point
  static constexpr char GO_TO_POINT_COMMAND_NAME[]{"GO TO POINT"};

  /// @brief The name of the command to turn to an angle
  static constexpr char TURN_TO_ANGLE_COMMAND_NAME[]{"TURN TO ANGLE"};

  /// @brief The name of the command to turn to a point
  static constexpr char TURN_TO_POINT_COMMAND_NAME[]{"TURN TO POINT"};

  /// @brief The name of the command to set the max velocity of the __drive
  /// straight__ algorithm
  static constexpr char SET_DRIVE_STRAIGHT_VELOCITY_COMMAND_NAME[]{
      "SET DRIVE STRAIGHT VELOCITY"};

  /// @brief The name of the command to set the max velocity of the __go to
  /// point__ algorithm
  static constexpr char SET_GO_TO_POINT_VELOCITY_COMMAND_NAME[]{
      "SET GO TO POINT VELOCITY"};

  /// @brief The name of the state determining if the __drive straight__
  /// algorithm has reached the target
  static constexpr char DRIVE_STRAIGHT_TARGET_REACHED_STATE_NAME[]{
      "DRIVE STRAIGHT TARGET REACHED"};

  /// @brief The name of the state determining if the __go to point__ algorithm
  /// has reached the target
  static constexpr char GO_TO_POINT_TARGET_REACHED_STATE_NAME[]{
      "GO TO POINT TARGET REACHED"};

  /// @brief The name of the state determining if the __turn__ algorithm has
  /// reached the target
  static constexpr char TURN_TARGET_REACHED_STATE_NAME[]{"TURN TARGET REACHED"};

  /// @brief The algorithm to drive straight
  std::unique_ptr<driftless::control::motion::IDriveStraight>
      m_drive_straight{};

  /// @brief @brief The algorithm to go to a point
  std::unique_ptr<driftless::control::motion::IGoToPoint> m_go_to_point{};

  /// @brief The algorithm to turn
  std::unique_ptr<driftless::control::motion::ITurn> m_turn{};

  /// @brief The current type of motion, defaults to __NONE__
  driftless::control::motion::EMotionType m_motion_type{EMotionType::NONE};

 public:
  /// @brief Constructs a new Motion Control object
  /// @param drive_straight - The algorithm used to drive straight
  /// @param go_to_point - The algorithm used to go to a point
  /// @param turn - The algorithm used to turn
  MotionControl(
      std::unique_ptr<driftless::control::motion::IDriveStraight>&
          drive_straight,
      std::unique_ptr<driftless::control::motion::IGoToPoint>& go_to_point,
      std::unique_ptr<driftless::control::motion::ITurn>& turn);

  /// @brief Initializes the motion control
  void init() override;

  /// @brief Runs the motion control
  void run() override;

  /// @brief Pauses the motion control
  void pause() override;

  /// @brief Resumes the motion control
  void resume() override;

  /// @brief Sends a command to the motion control
  /// @param command_name - The name of the command to run
  /// @param args - Any arguments needed for the command
  void command(std::string command_name, va_list& args) override;

  /// @brief Gets a state of the motion control
  /// @param state_name - The name of the state desired
  /// @return The desired state of the motion control
  void* state(std::string state_name) override;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif