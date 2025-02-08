#ifndef __PID_PATH_FOLLOWER_HPP__
#define __PID_PATH_FOLLOWER_HPP__

#include "driftless/control/PID.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/path/IPathFollower.hpp"
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

/// @brief Namespace for the path follower control
/// @author Matthew Backman
namespace path {

/// @brief Class representing a pure pursuit path follower using PID
/// @author Matthew Backman
class PIDPathFollower : public driftless::control::path::IPathFollower {
 private:
  // delay in ms between each task loop
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Constantly loops task updates
  /// @param params __void*__ Pointer to the taskUpdate function
  static void taskLoop(void* params);

  // delayer
  std::unique_ptr<driftless::rtos::IDelayer> m_delayer{};

  // mutex
  std::unique_ptr<driftless::rtos::IMutex> m_mutex{};

  // task for the algorithm
  std::unique_ptr<driftless::rtos::ITask> m_task{};

  // linear PID controller
  PID m_linear_pid{};

  // rotational PID controller
  PID m_rotational_pid{};

  // the "look ahead" distance for the robot
  double m_follow_distance{};

  // the tolerance for accepted values
  double m_target_tolerance{};

  // the acceptable velocity for being considered "at the target"
  double m_target_velocity{};

  // the robot
  std::shared_ptr<driftless::robot::Robot> m_robot{};

  // the path being followed
  std::vector<driftless::control::Point> m_control_path{};

  // the index of the latest point found
  uint32_t found_index{};

  // the robots velocity (in/s)
  double m_max_velocity{};

  // whether the algorithm is paused or running
  bool paused{};

  // whether the robot is at the target or not
  bool target_reached{true};

  /// @brief Updates the path follower algorithm
  void taskUpdate();

  /// @brief Sets the velocity of the drive train
  /// @param velocity robot::subsystems::drivetrain::Velocity__ The new velocity
  void setDriveVelocity(
      driftless::robot::subsystems::drivetrain::Velocity velocity);

  /// @brief Gets the radius of the drive train
  /// @return __double__ The radius of the drive train
  double getDriveRadius();

  /// @brief Gets the position from the odometry subsystem
  /// @return __robot::subsystems::odometry::Position__ The position of the
  /// robot
  driftless::robot::subsystems::odometry::Position getPosition();

  /// @brief Calculates the distance from the current position to the end of the
  /// path
  /// @param position __robot::subsystems::odometry::Position__ The current
  /// position
  /// @return __double__ The distance to the target
  double calculateDistanceToTarget(
      robot::subsystems::odometry::Position position);

  /// @brief Updates the points found
  /// @param position __robot::subsystems::odometry::Position__ The current
  /// position
  void updateFoundPoints(robot::subsystems::odometry::Position position);

  /// @brief Calculates the next follow point to target
  /// @param position __robot::subsystems::odometry::Position__ The current
  /// position
  /// @return __Point__ The next follow point
  Point calculateFollowPoint(robot::subsystems::odometry::Position position);

  /// @brief Updates the velocity of the drive train
  /// @param position __robot::subsystems::odometry::Position__ The current
  /// position
  /// @param follow_point __Point__ The target follow point
  void updateVelocity(robot::subsystems::odometry::Position position,
                      Point follow_point);

 public:
  /// @brief Initializes the path follower
  void init() override;

  /// @brief Runs the path follower
  void run() override;

  /// @brief Pauses the path follower
  void pause() override;

  /// @brief Resumes the path follower
  void resume() override;

  /// @brief Follows a given path
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param control_path __const std::vector<Point>&__ The list of control
  /// points
  /// @param velocity __double__ The maximum velocity
  void followPath(const std::shared_ptr<driftless::robot::Robot>& robot,
                  const std::vector<Point>& control_path,
                  double velocity) override;

  /// @brief Sets the max velocity to travel at
  /// @param velocity __double__ The new max velocity
  void setVelocity(double velocity) override;

  /// @brief Determines if the target has been reached
  /// @return __bool__ True if within the target range, false otherwise
  bool targetReached() override;

  /// @brief Sets the delayer used by the path follower
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer used
  void setDelayer(const std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  /// @brief Sets the mutex used by the path follower
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex used
  void setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  /// @brief Sets the task used by the path follower
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task used
  void setTask(std::unique_ptr<driftless::rtos::ITask>& task);

  /// @brief Sets the linear PID controller used by the path follower
  /// @param linear_pid __PID__ The linear PID controller used
  void setLinearPID(driftless::control::PID linear_pid);

  /// @brief Sets the rotational PID controller used by the path follower
  /// @param rotational_pid __PID__ The rotational PID controller used
  void setRotationalPID(driftless::control::PID rotational_pid);

  /// @brief Sets the follow distance used by the path follower
  /// @param follow_distance __double__ The follow distance used
  void setFollowDistance(double follow_distance);

  /// @brief Sets the target tolerance used by the path follower
  /// @param target_tolerance __double__ The target tolerance used
  void setTargetTolerance(double target_tolerance);

  /// @brief Sets the target velocity used by the path follower
  /// @param target_velocity __double__ The target velocity used
  void setTargetVelocity(double target_velocity);
};
}  // namespace path
}  // namespace control
}  // namespace driftless
#endif