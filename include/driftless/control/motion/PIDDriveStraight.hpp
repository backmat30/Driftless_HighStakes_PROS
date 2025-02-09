#ifndef __PID_DRIVE_STRAIGHT_HPP__
#define __PID_DRIVE_STRAIGHT_HPP__

#include <cmath>
#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/motion/IDriveStraight.hpp"
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

/// @brief Class representing a drive straight algorithm using PID controllers
/// @author Matthew Backman
class PIDDriveStraight : public driftless::control::motion::IDriveStraight {
 private:
  // delay on the task loop
  static constexpr uint8_t TASK_DELAY{10};

  // task loop to run background updates
  static void taskLoop(void* params);

  // the delayer
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // the linear pid controller
  PID m_linear_pid{};

  // the rotational pid controller
  PID m_rotational_pid{};

  // the tolerance for being at the target
  double m_target_tolerance{};

  // the max velocity for being considered at the target
  double m_target_velocity{};

  // the limit to the velocity output
  double m_max_velocity{};

  // the starting point
  Point m_starting_point{};

  // the target distance
  double m_target_distance{};

  // the target angle
  double m_target_angle{};

  // whether the robot has reached its current target
  bool target_reached{true};

  // whether the control has been paused
  bool paused{};

  /// @brief Sets the velocity of the drivetrain
  /// @param left __double__ The left wheel velocity
  /// @param right __double__ The right wheel velocity
  void setDriveVelocity(double left, double right);

  /// @brief Gets the position of the robot
  /// @return __robot::subsystems::odometry::Position__ The robot's position
  robot::subsystems::odometry::Position getPosition();

  /// @brief Gets the velocity of the robot
  /// @return __double__ The robot's velocity
  double getVelocity();

  /// @brief Updates the control velocity
  /// @param distance __double__ The distance to the target
  /// @param theta __double__ The angle to the target
  void updateVelocity(double distance, double theta);

  /// @brief Runs all updates
  void taskUpdate();

 public:
  /// @brief Initializes the control
  void init() override;

  /// @brief Runs the control
  void run() override;

  /// @brief Pauses the control
  void pause() override;

  /// @brief Resumes the control
  void resume() override;

  /// @brief Drives straight for a given distance in inches
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The velocity to drive at
  /// @param distance __double__ The distance to drive
  /// @param theta __double__ The angle to maintain
  void driveStraight(std::shared_ptr<driftless::robot::Robot>& robot,
                     double velocity, double distance, double theta) override;

  /// @brief Sets the velocity to run the control at
  /// @param velocity __double__ The velocity to set
  void setVelocity(double velocity) override;

  /// @brief Returns if the robot has reached the target
  /// @return __bool__ True if the target is reached, else false
  bool targetReached() override;

  /// @brief Sets the delayer
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The
  /// delayer
  void setDelayer(const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Sets the mutex
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex
  void setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Sets the task
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task
  void setTask(std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Sets the linear PID controller
  /// @param linear_pid __PID__ The linear PID controller
  void setLinearPID(PID linear_pid);

  /// @brief Sets the rotational PID controller
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