#ifndef __PID_GO_TO_POINT_HPP__
#define __PID_GO_TO_POINT_HPP__

#include <cmath>
#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/motion/IGoToPoint.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
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

/// @brief Class representing a go to point algorithm using PID
/// @author Matthew Backman
class PIDGoToPoint : public IGoToPoint {
 private:
  // the task delay
  static constexpr uint8_t TASK_DELAY{10};

  // task loop to run task updates
  static void taskLoop(void* params);

  // the delayer
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // the mutex
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // the task
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // the robot being controlled
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // the linear PID controller
  PID m_linear_pid{};

  // the rotational PID controller
  PID m_rotational_pid{};

  // the max velocity for motion
  double m_max_velocity{};

  // the tolerance for being at the target point
  double m_target_tolerance{};

  // the max velocity for being considerd at the target point
  double m_target_velocity{};

  // the target point
  Point m_target_point{};

  // whether the target has been reached
  bool target_reached{true};

  // whether the control is paused
  bool paused{};

  /// @brief Sets the velocity of the drive train
  /// @param left __double__ The desired left drive velocity
  /// @param right __double__ The desired right drive velocity
  void setDriveVelocity(double left, double right);

  /// @brief Gets the position of the robot from the odometry
  /// @return __robot::subsystems::odometry::Position__ The robot's position
  driftless::robot::subsystems::odometry::Position getPosition();

  /// @brief Gets the current velocity of the robot
  /// @return __double__ The robot's current velocity
  double getVelocity();

  /// @brief Updates the control velocity
  /// @param distance __double__ The distance to the target point
  /// @param target_angle __double__ The desired angle
  /// @param theta __double__ The current heading
  void updateVelocity(double distance, double target_angle, double theta);

  /// @brief Runs all instance specific updates
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

  /// @brief Tells the robot to go to a given point
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param velocity __double__ The max velocity to move at
  /// @param point __Point__ The point on the field to go to
  void goToPoint(const std::shared_ptr<driftless::robot::Robot>& robot,
                 double velocity, Point point) override;

  /// @brief Sets the max velocity to move at
  /// @param velocity __double__ The max velocity
  void setVelocity(double velocity) override;

  /// @brief Determines if the robot has reached the target
  /// @return __bool__ True if within the target range, else false
  bool targetReached() override;

  /// @brief Sets the delayer used
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer
  void setDelayer(std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Sets the mutex used
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex
  void setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Sets the task used
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task
  void setTask(std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Sets the linear PID controller used
  /// @param linear_pid __PID__ The linear PID controller
  void setLinearPID(PID linear_pid);

  /// @brief Sets the rotational PID controller used
  /// @param rotational_pid __PID__ The rotational PID controller
  void setRotationalPID(PID rotational_pid);

  /// @brief Sets the target tolerance
  /// @param target_tolerance __double__ The target tolerance
  void setTargetTolerance(double target_tolerance);

  /// @brief Sets the target velocity
  /// @param target_velocity __double__ The target velocity
  void setTargetVelocity(double targetVelocity);
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif