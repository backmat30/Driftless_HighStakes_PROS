#ifndef __MOTION_CONTROL_HPP__
#define __MOTION_CONTROL_HPP__

#include <memory>

#include "driftless/control/AControl.hpp"
#include "driftless/control/motion/EMotionType.hpp"
#include "driftless/control/motion/IDriveStraight.hpp"
#include "driftless/control/motion/IGoToPoint.hpp"
#include "driftless/control/motion/ITurn.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for basic motion control algorithms
/// @author Matthew Backman
namespace motion {

/// @brief Class to hold and control all motion algorithms
/// @author Matthew Backman
class MotionControl : public driftless::control::AControl {
 private:
  /// @brief The algorithm to drive straight
  std::unique_ptr<driftless::control::motion::IDriveStraight>
      m_drive_straight{};

  /// @brief The algorithm to go to a point
  std::unique_ptr<driftless::control::motion::IGoToPoint> m_go_to_point{};

  /// @brief The algorithm to turn
  std::unique_ptr<driftless::control::motion::ITurn> m_turn{};

  /// @brief The current type of motion, defaults to __NONE__
  driftless::control::motion::EMotionType m_motion_type{EMotionType::NONE};

 public:
  /// @brief Constructs a new Motion Control object
  /// @param drive_straight __std::unique_ptr<IDriveStraight>&__ The algorithm
  /// used to drive straight
  /// @param go_to_point __std::unique_ptr<IGoToPoint>&__ The algorithm used to
  /// go to a point
  /// @param turn __std::unique_ptr<ITurn>&__ The algorithm used to turn
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
  /// @param command_name __EControlCommand__ The name of the command to run
  /// @param args __va_list&__ Any arguments needed for the command
  void command(EControlCommand command_name, va_list& args) override;

  /// @brief Gets a state of the motion control
  /// @param state_name __EControlState__ The name of the state desired
  /// @return __void*__ The desired state of the motion control
  void* state(EControlState state_name) override;
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif