#ifndef __PID_BOOMERANG_HPP__
#define __PID_BOOMERANG_HPP__

#include <cmath>

#include "driftless/control/PID.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/boomerang/IBoomerang.hpp"
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

/// @brief Namespace for the boomerang control
/// @author Matthew Backman
namespace boomerang {

/// @brief Class representing a boomerang control using PID
/// @author Matthew Backman
class PIDBoomerang : public IBoomerang {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Constantly updates the boomerang control
  /// @param params __void*__ The PIDBoomerang being updated
  static void taskLoop(void* params);

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  PID m_linear_pid{};

  PID m_rotational_pid{};

  double m_lead{};

  double m_aim_distance{};

  double m_target_tolerance{};

  double m_target_velocity{};

  std::shared_ptr<robot::Robot> control_robot{};

  double max_velocity{};

  double target_x{};

  double target_y{};

  double target_theta{};

  bool paused{};

  bool target_reached{true};

  /// @brief Updates the boomerang control
  void taskUpdate();

  /// @brief Sets the velocity of the drive train
  /// @param velocity __robot::subsystems::drivetrain::Velocity__ The new
  /// velocity
  void setDriveVelocity(robot::subsystems::drivetrain::Velocity velocity);

  /// @brief Gets the position of the robot from the odometry
  /// @return __robot::subsystems::odometry::Position__ The position of the
  /// robot
  robot::subsystems::odometry::Position getOdomPosition() const;

  /// @brief Calculates the distance to the target point
  /// @param position __robot::subsystems::odometry::Position__ The current
  /// position of the
  /// @return __double__ The distance to the target
  double calculateDistance(
      robot::subsystems::odometry::Position position) const;

  /// @brief Calculates the carrot point
  /// @param distance __double__ The distance to the target
  /// @return __Point__ The carrot point
  Point calculateCarrotPoint(double distance) const;

  /// @brief Calculates the velocity to travel at
  /// @param position __robot::subsystems::odometry::Position__ The current
  /// position
  /// @param carrot_point __Point__ The carrot point
  void updateVelocity(robot::subsystems::odometry::Position position,
                      Point carrot_point);

 public:
  /// @brief Initializes the PIDBoomerang control
  void init() override;

  /// @brief Runs the PIDBoomerang control
  void run() override;

  /// @brief Goes to a given position
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot being
  /// controlled
  /// @param velocity __double__ The maximum velocity
  /// @param x __double__ The target x position
  /// @param y __double__ The target y position
  /// @param theta __double__ The target angle
  void goToPosition(const std::shared_ptr<robot::Robot>& robot, double velocity,
                    double x, double y, double theta) override;

  /// @brief Sets the max velocity of the boomerang control
  /// @param velocity __double__ The max velocity
  void setVelocity(double velocity) override;

  /// @brief Pauses the boomerang control
  void pause() override;

  /// @brief Resumes the boomerang control
  void resume() override;

  /// @brief Determines if the target has been reached
  /// @return __bool__ True if the target has been reached, false otherwise
  bool targetReached() override;

  /// @brief Sets the delayer to use
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer to use
  void setDelayer(const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex to use
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to use
  void setMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task to use
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to use
  void setTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Sets the PID controller to use for linear control
  /// @param linearPID __PID__ The PID controller to use
  void setLinearPID(PID linearPID);

  /// @brief Sets the PID controller to use for rotational control
  /// @param rotationalPID __PID__ The PID controller to use
  void setRotationalPID(PID rotationalPID);

  /// @brief Sets the lead distance
  /// @param lead __double__ The lead distance to use
  void setLead(double lead);

  /// @brief Sets the aim distance
  /// @param aim_distance __double__ The aim distance to use
  void setAimDistance(double aim_distance);

  /// @brief Sets the tolerance distance around the target point
  /// @param target_tolerance __double__ The target tolerance to use
  void setTargetTolerance(double target_tolerance);

  /// @brief Sets the acceptable velocity for the target to be reached
  /// @param target_velocity __double__ The target velocity to use
  void setTargetVelocity(double target_velocity);
};
}  // namespace boomerang
}  // namespace control
}  // namespace driftless
#endif