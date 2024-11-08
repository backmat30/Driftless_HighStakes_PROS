#ifndef __PID_GO_TO_POINT_HPP__
#define __PID_GO_TO_POINT_HPP__

#include <cmath>
#include <memory>

#include "pvegas/control/PID.hpp"
#include "pvegas/control/Point.hpp"
#include "pvegas/control/motion/IGoToPoint.hpp"
#include "pvegas/robot/subsystems/odometry/Position.hpp"
#include "pvegas/rtos/IDelayer.hpp"
#include "pvegas/rtos/IMutex.hpp"
#include "pvegas/rtos/ITask.hpp"
#include "pvegas/utils/UtilityFunctions.hpp"

namespace driftless {
namespace control {
namespace motion {
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

  // sets the velocity of the drive train
  void setDriveVelocity(double left, double right);

  // gets the position of the robot from the odometry
  driftless::robot::subsystems::odometry::Position getPosition();

  // gets the current velocity of the robot
  double getVelocity();

  // updates the control velocity
  void updateVelocity(double distance, double target_angle, double theta);

  // run all instance specific updates
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

  // tell the robot to go to a given point
  void goToPoint(const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
                 Point point) override;

  // set the max velocity to move at
  void setVelocity(double velocity) override;

  // determines if the robot has reached the target
  bool targetReached() override;

  // set the delayer
  void setDelayer(std::unique_ptr<driftless::rtos::IDelayer>& delayer);

  // set the mutex
  void setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex);

  // set the task
  void setTask(std::unique_ptr<driftless::rtos::ITask>& task);

  // set the linear pid controller
  void setLinearPID(PID linear_pid);

  // set the rotational pid controller
  void setRotationalPID(PID rotational_pid);

  // set the target tolerance
  void setTargetTolerance(double target_tolerance);

  // set the target velocity
  void setTargetVelocity(double targetVelocity);
};
}  // namespace motion
}  // namespace control
}  // namespace pvegas
#endif