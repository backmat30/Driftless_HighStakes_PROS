#ifndef __PID_TURN_HPP__
#define __PID_TURN_HPP__

#include <cmath>
#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/control/motion/ITurn.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/drivetrain/Velocity.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for direct motion control
/// @author Matthew Backman
namespace motion {

/// @brief Class representing a turn algorithm using a PID controller
/// @author Matthew Backman
class PIDTurn : public ITurn {
 private:
  // the task delay
  static constexpr uint8_t TASK_DELAY{10};

  // the distance to the imaginary point to turn towards
  static constexpr double TURN_TO_ANGLE_DISTANCE{120000};

  /// @brief Constantly runs task updates
  /// @param params __void*__ Pointer to the PIDTurn object being updated
  static void taskLoop(void* params);

  // the delayer
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // the rotational PID controller
  PID m_rotational_pid{};

  // the max velocity for turning
  double m_max_velocity{};

  // the direction to turn in
  ETurnDirection m_turn_direction{};

  // the tolerance for being at the target point
  double m_target_tolerance{};

  // the max velocity for being considerd at the target point
  double m_target_velocity{};

  // the target point
  Point m_target_point{};

  // whether the target has been reached
  bool target_reached{true};

  // whether the forced direction has been reached
  bool forced_direction_reached{};

  // whether the control is paused
  bool paused{};

  /// @brief Sets the velocity of the drive train
  /// @param velocity __robot::subsystems::drivetrain::Velocity__ The desired
  /// drive velocity
  void setDriveVelocity(
      driftless::robot::subsystems::drivetrain::Velocity velocity);

  /// @brief Gets the position of the robot from the odometry
  /// @return __robot::subsystems::odometry::Position__ The robot's position
  driftless::robot::subsystems::odometry::Position getPosition();

  /// @brief Gets the radius of the drive train
  /// @return __double__ The drive train's radius
  double getDriveRadius();

  /// @brief Calculates the angle between the current heading and the target
  /// point
  /// @param position __robot::subsystems::odometry::Position__ The current
  /// position
  /// @return __double__ The angle, in radians
  double calculateAngleToTarget(
      driftless::robot::subsystems::odometry::Position position);

  /// @brief Calculates the velocity to pass to the drive train
  /// @param current_angle __double__ The robot's current heading
  /// @param target_angle __double__ The desired angle
  /// @return __robot::subsystems::drivetrain::Velocity__ The velocity for the
  /// drive train
  robot::subsystems::drivetrain::Velocity calculateDriveVelocity(
      double current_angle, double target_angle);

  /// @brief Runs all instance related updates
  void taskUpdate();

 public:
  /// @brief Initializes the PIDTurn
  void init() override;

  /// @brief Runs the PIDTurn
  void run() override;

  /// @brief Pauses the PIDTurn
  void pause() override;

  /// @brief Resumes the PIDTurn
  void resume() override;

  /// @brief Turns towards a designated angle, in radians
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The max velocity to move at
  /// @param theta __double__ The desired angle
  /// @param direction __ETurnDirection__ The direction to turn in; leave blank
  /// to take shortest path
  void turnToAngle(const std::shared_ptr<driftless::robot::Robot>& robot,
                   double velocity, double theta,
                   ETurnDirection direction = ETurnDirection::AUTO) override;

  /// @brief Turns towards a point on the field
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The max velocity to move at
  /// @param point __Point__ The point on the field to turn towards
  /// @param direction __ETurnDirection__ The direction to turn in; leave blank
  /// to take shortest path
  void turnToPoint(const std::shared_ptr<driftless::robot::Robot>& robot,
                   double velocity, Point point,
                   ETurnDirection direction = ETurnDirection::AUTO) override;

  /// @brief Determines if the target angle has been reached
  /// @return __bool__ True if within the target range, else false
  bool targetReached() override;

  /// @brief Sets the delayer used
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer
  void setDelayer(const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Sets the mutex used
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex
  void setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Sets the task used
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task
  void setTask(std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Sets the rotational PID controller used
  /// @param rotational_pid __PID__ The rotational PID controller
  void setRotationalPID(PID rotational_pid);

  /// @brief Sets the target tolerance
  /// @param target_tolerance __double__ The target tolerance
  void setTargetTolerance(double target_tolerance);

  /// @brief Sets the target velocity
  /// @param target_velocity __double__ The target velocity
  void setTargetVelocity(double target_velocity);
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif