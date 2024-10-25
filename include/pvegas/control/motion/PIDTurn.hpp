#ifndef __PID_TURN_HPP__
#define __PID_TURN_HPP__

#include <cmath>
#include <memory>

#include "pvegas/control/PID.hpp"
#include "pvegas/control/motion/ITurn.hpp"
#include "pvegas/robot/subsystems/drivetrain/Velocity.hpp"
#include "pvegas/robot/subsystems/odometry/Position.hpp"
#include "pvegas/rtos/IDelayer.hpp"
#include "pvegas/rtos/IMutex.hpp"
#include "pvegas/rtos/ITask.hpp"
#include "pvegas/utils/UtilityFunctions.hpp"

namespace pvegas {
namespace control {
namespace motion {
class PIDTurn : public ITurn {
 private:
  // the task delay
  static constexpr uint8_t TASK_DELAY{10};

  // the distance to the imaginary point to turn towards
  static constexpr double TURN_TO_ANGLE_DISTANCE{120000};

  // task loop to run task updates
  static void taskLoop(void* params);

  // the delayer
  std::unique_ptr<pvegas::rtos::IDelayer> m_delayer{};

  // the mutex
  std::unique_ptr<pvegas::rtos::IMutex> m_mutex{};

  // the task
  std::unique_ptr<pvegas::rtos::ITask> m_task{};

  // the robot being controlled
  std::shared_ptr<pvegas::robot::Robot> m_robot{};

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

  // sets the velocity of the drive train
  void setDriveVelocity(pvegas::robot::subsystems::drivetrain::Velocity velocity);

  // gets the position of the robot from the odometry
  pvegas::robot::subsystems::odometry::Position getPosition();

  // gets the radius of the drive train
  double getDriveRadius();

  // calculates the angle from the given position to the target position
  double calculateAngleToTarget(
      pvegas::robot::subsystems::odometry::Position position);

  // calculates the velocity for the drive train
  robot::subsystems::drivetrain::Velocity calculateDriveVelocity(
      double current_angle, double target_angle);

  // runs all instance related updates
  void taskUpdate();

 public:
  // initialize the control
  void init() override;

  // run the control
  void run() override;

  // pause the control
  void pause() override;

  // resume the control
  void resume() override;

  // tell the robot to turn in to a given angle
  void turnToAngle(const std::shared_ptr<pvegas::robot::Robot>& robot,
                   double velocity, double theta,
                   ETurnDirection direction = ETurnDirection::AUTO) override;

  // tell the robot to turn towards a point on the field
  void turnToPoint(const std::shared_ptr<pvegas::robot::Robot>& robot,
                   double velocity, Point point,
                   ETurnDirection direction = ETurnDirection::AUTO) override;

  // determines if the robot has reached the target
  bool targetReached() override;

  // set the delayer
  void setDelayer(const std::unique_ptr<pvegas::rtos::IDelayer>& delayer);

  // set the mutex
  void setMutex(std::unique_ptr<pvegas::rtos::IMutex>& mutex);

  // set the task
  void setTask(std::unique_ptr<pvegas::rtos::ITask>& task);

  // sets the rotational pid controller
  void setRotationalPID(PID rotational_pid);

  // set the target tolerance
  void setTargetTolerance(double target_tolerance);

  // set the target velocity
  void setTargetVelocity(double target_velocity);
};
}  // namespace motion
}  // namespace control
}  // namespace pvegas
#endif