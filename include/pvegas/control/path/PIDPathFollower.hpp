#ifndef __PID_PATH_FOLLOWER_HPP__
#define __PID_PATH_FOLLOWER_HPP__

#include "pvegas/control/PID.hpp"
#include "pvegas/control/path/IPathFollower.hpp"
#include "pvegas/robot/subsystems/drivetrain/Velocity.hpp"
#include "pvegas/robot/subsystems/odometry/Position.hpp"
#include "pvegas/rtos/IDelayer.hpp"
#include "pvegas/rtos/IMutex.hpp"
#include "pvegas/rtos/ITask.hpp"
#include "pvegas/utils/UtilityFunctions.hpp"

namespace pvegas {
namespace control {
namespace path {
class PIDPathFollower : public pvegas::control::path::IPathFollower {
 private:
  // delay in ms between each task loop
  static constexpr uint8_t TASK_DELAY{10};

  // name of the drive train subsystem
  static constexpr char DRIVE_SUBSYSTEM_NAME[]{"DIFFERENTIAL DRIVE"};

  // name of the odometry subsystem
  static constexpr char ODOMETRY_SUBSYSTEM_NAME[]{"POSITION TRACKER"};

  // name of the drive train set velocity command
  static constexpr char DRIVE_SET_VELOCITY_COMMAND_NAME[]{"SET VELOCITY"};

  // name of the drive train get drive radius command
  static constexpr char DRIVE_GET_RADIUS_COMMAND_NAME[]{"GET RADIUS"};

  // name of the odometry get position command
  static constexpr char ODOMETRY_GET_POSITION_COMMAND_NAME[]{"GET POSITION"};

  // background task loop to update algorithm
  static void taskLoop(void* params);

  // delayer
  std::unique_ptr<pvegas::rtos::IDelayer> m_delayer{};

  // mutex
  std::unique_ptr<pvegas::rtos::IMutex> m_mutex{};

  // task for the algorithm
  std::unique_ptr<pvegas::rtos::ITask> m_task{};

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
  std::shared_ptr<pvegas::robot::Robot> m_robot{};

  // the path being followed
  std::vector<pvegas::control::Point> m_control_path{};

  // the index of the latest point found
  uint32_t found_index{};

  // the robots velocity (in/s)
  double m_max_velocity{};

  // whether the algorithm is paused or running
  bool paused{};

  // whether the robot is at the target or not
  bool target_reached{true};

  // updates the task
  void taskUpdate();

  // set the velocity of the drive train
  void setDriveVelocity(
      pvegas::robot::subsystems::drivetrain::Velocity velocity);

  // get the radius of the drive train
  double getDriveRadius();

  // gets the position of the robot from the odometry subsystem
  pvegas::robot::subsystems::odometry::Position getPosition();

  // calculates the distance to the end of the path
  double calculateDistanceToTarget(
      robot::subsystems::odometry::Position position);

  // updates the points found
  void updateFoundPoints(robot::subsystems::odometry::Position position);

  // calculates the follow point, aka where the robot will go on the current
  // iteration
  Point calculateFollowPoint(robot::subsystems::odometry::Position position);

  // update the drive velocity
  void updateVelocity(robot::subsystems::odometry::Position position,
                      Point follow_point);

 public:
  // initialize the path follower
  void init() override;

  // run the path follower
  void run() override;

  // pause the path follower
  void pause() override;

  // resume the path follower
  void resume() override;

  // follow a given path of points
  void followPath(const std::shared_ptr<pvegas::robot::Robot>& robot,
                  const std::vector<Point>& control_path, double velocity) override;

  // sets the velocity to follow the path at
  void setVelocity(double velocity) override;

  // check if the target was reached
  bool targetReached() override;

  // set the delayer
  void setDelayer(const std::unique_ptr<pvegas::rtos::IDelayer>& delayer);

  // set the mutex
  void setMutex(std::unique_ptr<pvegas::rtos::IMutex>& mutex);

  // set the task
  void setTask(std::unique_ptr<pvegas::rtos::ITask>& task);

  // set the linear PID controller
  void setLinearPID(pvegas::control::PID linear_pid);

  // set the rotational PID controller
  void setRotationalPID(pvegas::control::PID rotational_pid);

  // set the follow distance
  void setFollowDistance(double follow_distance);

  // sets the tolerance for reaching the target
  void setTargetTolerance(double target_tolerance);

  // sets the maximum velocity for the robot to be considered at the target
  void setTargetVelocity(double target_velocity);
};
}  // namespace path
}  // namespace control
}  // namespace pvegas
#endif