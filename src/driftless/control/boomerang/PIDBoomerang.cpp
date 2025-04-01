#include "driftless/control/boomerang/PIDBoomerang.hpp"

namespace driftless::control::boomerang {
void PIDBoomerang::taskLoop(void* params) {
  while (true) {
    PIDBoomerang* boomerang{static_cast<PIDBoomerang*>(params)};
    boomerang->taskUpdate();
  }
}

void PIDBoomerang::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!paused && !target_reached) {
    robot::subsystems::odometry::Position position{getOdomPosition()};
    double distance{calculateDistance(position)};
    double velocity{
        std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2))};

    if (distance < m_target_tolerance && velocity < m_target_velocity) {
      target_reached = true;
      robot::subsystems::drivetrain::Velocity stopped{0, 0};
      setDriveVelocity(stopped);

    } else {
      Point carrot_point{calculateCarrotPoint(distance)};
      updateVelocity(position, carrot_point);
    }
  }
  if (m_mutex) {
    m_mutex->give();
  }

  m_delayer->delay(TASK_DELAY);
}

void PIDBoomerang::setDriveVelocity(
    robot::subsystems::drivetrain::Velocity velocity) {
  control_robot->sendCommand(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemCommand::DRIVETRAIN_SET_VELOCITY, velocity);
}

robot::subsystems::odometry::Position PIDBoomerang::getOdomPosition() const {
  robot::subsystems::odometry::Position position{};

  robot::subsystems::odometry::Position* odom_state{
      static_cast<robot::subsystems::odometry::Position*>(
          control_robot->getState(
              robot::subsystems::ESubsystem::ODOMETRY,
              robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  if (odom_state) {
    position = *odom_state;
    delete odom_state;
  }

  return position;
}

double PIDBoomerang::calculateDistance(
    robot::subsystems::odometry::Position position) const {
  double distance{std::sqrt(std::pow(position.x - target_x, 2) +
                            std::pow(position.y - target_y, 2))};
  return distance;
}

Point PIDBoomerang::calculateCarrotPoint(double distance) const {
  double carrot_x{target_x - (distance * std::cos(target_theta)) * m_lead};
  double carrot_y{target_y - (distance * std::sin(target_theta)) * m_lead};
  Point carrot_point{carrot_x, carrot_y};

  return carrot_point;
}

void PIDBoomerang::updateVelocity(
    robot::subsystems::odometry::Position position, Point carrot_point) {
  double x_error{carrot_point.getX() - position.x};
  double y_error{carrot_point.getY() - position.y};

  double rotational_error{
      bindRadians(std::atan2(y_error, x_error) - position.theta)};
  double linear_error{std::cos(rotational_error) *
                      std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2))};

  if (linear_error < 0) {
    rotational_error = bindRadians(rotational_error + M_PI);
  }

  double linear_control{m_linear_pid.getControlValue(0, linear_error)};
  if (std::abs(linear_control) > max_velocity) {
    linear_control = max_velocity * (linear_control / std::abs(linear_control));
  }
  double rotational_control{};
  if (calculateDistance(position) > m_aim_distance) {
    rotational_control = m_rotational_pid.getControlValue(0, rotational_error);
  }

  double left_velocity{linear_control - rotational_control};
  double right_velocity{linear_control + rotational_control};
  robot::subsystems::drivetrain::Velocity velocity{left_velocity,
                                                   right_velocity};
  setDriveVelocity(velocity);
}

void PIDBoomerang::init() {
  m_linear_pid.reset();
  m_rotational_pid.reset();
}

void PIDBoomerang::run() { m_task->start(&PIDBoomerang::taskLoop, this); }

void PIDBoomerang::goToPosition(const std::shared_ptr<robot::Robot>& robot,
                                double velocity, double x, double y,
                                double theta) {
  if (m_mutex) {
    m_mutex->take();
  }
  control_robot = robot;
  target_x = x;
  target_y = y;
  target_theta = theta;
  max_velocity = velocity;
  target_reached = false;
  paused = false;

  robot::subsystems::odometry::Position position{getOdomPosition()};
  double angle_to_target{angle(position.x, position.y, target_x, target_y)};
  double angle_error{bindRadians(angle_to_target - position.theta)};
  if (std::abs(angle_error) > M_PI / 2.0) {
    target_theta = bindRadians(target_theta + M_PI);
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDBoomerang::setVelocity(double velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  max_velocity = velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDBoomerang::pause() {
  if (m_mutex) {
    m_mutex->take();
  }

  paused = true;
  robot::subsystems::drivetrain::Velocity stopped{0, 0};
  setDriveVelocity(stopped);

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDBoomerang::resume() {
  if (m_mutex) {
    m_mutex->take();
  }

  paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDBoomerang::targetReached() { return target_reached; }

void PIDBoomerang::setDelayer(const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDBoomerang::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDBoomerang::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDBoomerang::setLinearPID(PID linear_pid) { m_linear_pid = linear_pid; }

void PIDBoomerang::setRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDBoomerang::setLead(double lead) { m_lead = lead; }

void PIDBoomerang::setAimDistance(double aim_distance) {
  m_aim_distance = aim_distance;
}

void PIDBoomerang::setTargetTolerance(double target_tolerance) {
  m_target_tolerance = target_tolerance;
}

void PIDBoomerang::setTargetVelocity(double target_velocity) {
  m_target_velocity = target_velocity;
}
}  // namespace driftless::control::boomerang