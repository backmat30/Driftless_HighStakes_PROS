#include "driftless/control/motion/PIDDriveStraight.hpp"

#include "pros/screen.hpp"
namespace driftless {
namespace control {
namespace motion {
void PIDDriveStraight::taskLoop(void* params) {
  PIDDriveStraight* pid_drive_straight{static_cast<PIDDriveStraight*>(params)};

  while (true) {
    pid_drive_straight->taskUpdate();
  }
}

void PIDDriveStraight::setDriveVelocity(double left, double right) {
  if (m_robot) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::DRIVETRAIN,
        robot::subsystems::ESubsystemCommand::DRIVETRAIN_SET_VELOCITY, left,
        right);
  }
}

driftless::robot::subsystems::odometry::Position
PIDDriveStraight::getPosition() {
  driftless::robot::subsystems::odometry::Position position{};
  if (m_robot) {
    position = *static_cast<driftless::robot::subsystems::odometry::Position*>(
        m_robot->getState(
            robot::subsystems::ESubsystem::ODOMETRY,
            robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
  }
  return position;
}

double PIDDriveStraight::getVelocity() {
  driftless::robot::subsystems::odometry::Position position{getPosition()};
  return std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2));
}

void PIDDriveStraight::updateVelocity(double distance, double theta) {
  double linear_control{m_linear_pid.getControlValue(0, distance)};
  double angular_control{
      m_rotational_pid.getControlValue(0, bindRadians(m_target_angle - theta))};

  if (std::abs(linear_control) > m_max_velocity) {
    linear_control *= m_max_velocity / std::abs(linear_control);
  }

  double current_velocity{getVelocity()};

  double left_velocity{linear_control - angular_control};
  double right_velocity{linear_control + angular_control};

  setDriveVelocity(left_velocity, right_velocity);
}

void PIDDriveStraight::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!target_reached && !paused) {
    driftless::robot::subsystems::odometry::Position position{getPosition()};

    double start_distance{distance(m_starting_point.getX(),
                                   m_starting_point.getY(), position.x,
                                   position.y)};
    double start_angle{angle(m_starting_point.getX(), m_starting_point.getY(),
                             position.x, position.y)};

    if (m_target_distance < 0) {
      start_distance *= -1;
      start_angle = bindRadians(start_angle + M_PI);
    }

    double distance{
        m_target_distance -
        (start_distance * std::cos(bindRadians(start_angle - m_target_angle)))};
    double velocity{getVelocity()};

    if (std::abs(distance) < m_target_tolerance &&
        velocity < m_target_velocity) {
      target_reached = true;
    } else {
      updateVelocity(distance, position.theta);
    }
  }

  if (m_mutex) {
    m_mutex->give();
  }
  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

void PIDDriveStraight::init() {
  m_linear_pid.reset();
  m_rotational_pid.reset();
}

void PIDDriveStraight::run() {
  if (m_task) {
    m_task->start(&PIDDriveStraight::taskLoop, this);
  }
}

void PIDDriveStraight::pause() {
  if (m_mutex) {
    m_mutex->take();
  }
  paused = true;
  setDriveVelocity(0, 0);
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDDriveStraight::resume() {
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

void PIDDriveStraight::driveStraight(
    std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
    double distance, double theta) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_linear_pid.reset();
  m_rotational_pid.reset();
  m_robot = robot;
  m_max_velocity = velocity;
  driftless::robot::subsystems::odometry::Position position{getPosition()};
  m_starting_point.setX(position.x);
  m_starting_point.setY(position.y);
  m_target_distance = distance;
  m_target_angle = theta;
  target_reached = false;
  paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDDriveStraight::setVelocity(double velocity) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_max_velocity = velocity;
  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDDriveStraight::targetReached() { return target_reached; }

void PIDDriveStraight::setDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDDriveStraight::setMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDDriveStraight::setTask(std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDDriveStraight::setLinearPID(PID linear_pid) {
  m_linear_pid = linear_pid;
}

void PIDDriveStraight::setRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDDriveStraight::setTargetTolerance(double target_tolerance) {
  m_target_tolerance = target_tolerance;
}

void PIDDriveStraight::setTargetVelocity(double target_velocity) {
  m_target_velocity = target_velocity;
}
}  // namespace motion
}  // namespace control
}  // namespace driftless