#include "driftless/control/motion/PIDGoToPoint.hpp"

namespace driftless {
namespace control {
namespace motion {
void PIDGoToPoint::taskLoop(void* params) {
  PIDGoToPoint* pid_go_to_point{static_cast<PIDGoToPoint*>(params)};

  while (true) {
    pid_go_to_point->taskUpdate();
  }
}

void PIDGoToPoint::setDriveVelocity(double left, double right) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemCommand::DRIVETRAIN_SET_VELOCITY, left,
      right);
}

driftless::robot::subsystems::odometry::Position PIDGoToPoint::getPosition() {
  driftless::robot::subsystems::odometry::Position position{};
  position = *static_cast<driftless::robot::subsystems::odometry::Position*>(
      m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
  return position;
}

double PIDGoToPoint::getVelocity() {
  driftless::robot::subsystems::odometry::Position position{getPosition()};
  return std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2));
}

void PIDGoToPoint::updateVelocity(double distance, double target_angle,
                                  double theta) {
  double linear_control{m_linear_pid.getControlValue(0, distance)};
  if (std::abs(linear_control) > m_max_velocity) {
    linear_control *= m_max_velocity / std::abs(linear_control);
  }

  double current_velocity{getVelocity()};

  double angular_offset{bindRadians(target_angle - theta)};
  double angular_error{};
  if (angular_offset < 0) {
    angular_error = ((2 / M_PI) * std::abs(angular_offset + (M_PI / 2))) - 1;
  } else if (angular_offset > 0) {
    angular_error = ((-2 / M_PI) * std::abs(angular_offset - (M_PI / 2))) + 1;
  } else {
    angular_error = 0;
  }

  double rotational_control{m_rotational_pid.getControlValue(angular_error, 0)};

  double left_velocity{linear_control * (1 + rotational_control)};
  double right_velocity{linear_control * (1 - rotational_control)};

  setDriveVelocity(left_velocity, right_velocity);
}

void PIDGoToPoint::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!target_reached && !paused) {
    driftless::robot::subsystems::odometry::Position position{getPosition()};

    double target_distance{distance(
        position.x, position.y, m_target_point.getX(), m_target_point.getY())};
    double target_angle{angle(position.x, position.y, m_target_point.getX(),
                              m_target_point.getY())};
    double forward_distance{target_distance *
                            std::cos(target_angle - position.theta)};
    double velocity{getVelocity()};

    if (std::abs(forward_distance) < m_target_tolerance &&
        velocity < m_target_velocity) {
      target_reached = true;
    } else {
      updateVelocity(forward_distance, target_angle, position.theta);
    }
  }

  if (m_mutex) {
    m_mutex->give();
  }
  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

void PIDGoToPoint::init() {
  m_linear_pid.reset();
  m_rotational_pid.reset();
}

void PIDGoToPoint::run() {
  if (m_task) {
    m_task->start(&PIDGoToPoint::taskLoop, this);
  }
}

void PIDGoToPoint::pause() {
  if (m_mutex) {
    m_mutex->take();
  }
  paused = true;
  setDriveVelocity(0, 0);
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDGoToPoint::resume() {
  if (m_mutex) {
    m_mutex->take();
  }
  paused = false;
  m_linear_pid.reset();
  m_rotational_pid.reset();
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDGoToPoint::goToPoint(
    const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
    Point target) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_linear_pid.reset();
  m_rotational_pid.reset();
  m_robot = robot;
  m_max_velocity = velocity;
  m_target_point = target;
  target_reached = false;
  paused = false;
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDGoToPoint::setVelocity(double velocity) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_max_velocity = velocity;
  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDGoToPoint::targetReached() { return target_reached; }

void PIDGoToPoint::setDelayer(
    std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void PIDGoToPoint::setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDGoToPoint::setTask(std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDGoToPoint::setLinearPID(PID linear_pid) { m_linear_pid = linear_pid; }

void PIDGoToPoint::setRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDGoToPoint::setTargetTolerance(double target_tolerance) {
  m_target_tolerance = target_tolerance;
}

void PIDGoToPoint::setTargetVelocity(double target_velocity) {
  m_target_velocity = target_velocity;
}
}  // namespace motion
}  // namespace control
}  // namespace driftless