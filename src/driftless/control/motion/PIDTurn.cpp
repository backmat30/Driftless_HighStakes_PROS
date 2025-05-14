#include "driftless/control/motion/PIDTurn.hpp"

namespace driftless {
namespace control {
namespace motion {
void PIDTurn::taskLoop(void* params) {
  PIDTurn* pid_turn{static_cast<PIDTurn*>(params)};

  while (true) {
    pid_turn->taskUpdate();
  }
}

void PIDTurn::setDriveVelocity(
    driftless::robot::subsystems::drivetrain::Velocity velocity) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemCommand::DRIVETRAIN_SET_VELOCITY, velocity);
}

driftless::robot::subsystems::odometry::Position PIDTurn::getPosition() {
  driftless::robot::subsystems::odometry::Position position{};
  position = *static_cast<driftless::robot::subsystems::odometry::Position*>(
      m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
  return position;
}

double PIDTurn::getDriveRadius() {
  double drive_radius{};
  drive_radius = *static_cast<double*>(m_robot->getState(
      robot::subsystems::ESubsystem::DRIVETRAIN,
      robot::subsystems::ESubsystemState::DRIVETRAIN_GET_RADIUS));
  return drive_radius;
}

double PIDTurn::calculateAngleToTarget(
    driftless::robot::subsystems::odometry::Position position) {
  double x_error{m_target_point.getX() - position.x};
  double y_error{m_target_point.getY() - position.y};
  return std::atan2(y_error, x_error);
}

robot::subsystems::drivetrain::Velocity PIDTurn::calculateDriveVelocity(
    double robot_angle, double target_angle) {
  double angle_error{bindRadians(target_angle - robot_angle)};

  if (!forced_direction_reached) {
    // set the angle to negative if direction should be clockwise, otherwise
    // make the angle positive
    if (m_turn_direction == ETurnDirection::CLOCKWISE && angle_error > 0) {
      angle_error -= 2 * M_PI;
    } else if (m_turn_direction == ETurnDirection::COUNTERCLOCKWISE &&
               angle_error < 0) {
      angle_error += 2 * M_PI;
    }
    // if robot is within 1/4 of a rotation of the destination, confirm that the
    // forced direction has been achieved
    if (std::abs(angle_error) < M_PI / 2) {
      forced_direction_reached = true;
    }
  }

  double rotational_control{m_rotational_pid.getControlValue(0, angle_error)};
  if (std::abs(rotational_control) > m_max_velocity) {
    rotational_control *= m_max_velocity / std::abs(rotational_control);
  }
  robot::subsystems::drivetrain::Velocity velocity{-rotational_control,
                                                   rotational_control};
  return velocity;
}

void PIDTurn::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!target_reached && !paused) {
    driftless::robot::subsystems::odometry::Position position{getPosition()};

    double target_angle{calculateAngleToTarget(position)};
    double angle_error{bindRadians(target_angle - position.theta)};

    robot::subsystems::drivetrain::Velocity velocity{};
    if (std::abs(angle_error) < m_target_tolerance &&
        std::abs(position.thetaV) < m_target_velocity) {
      target_reached = true;
      velocity.left_velocity = 0;
      velocity.right_velocity = 0;
    } else {
      velocity = calculateDriveVelocity(position.theta, target_angle);
    }
    setDriveVelocity(velocity);
  }
  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

void PIDTurn::init() { m_rotational_pid.reset(); }

void PIDTurn::run() {
  if (m_task) {
    m_task->start(&PIDTurn::taskLoop, this);
  }
}

void PIDTurn::pause() {
  if (m_mutex) {
    m_mutex->take();
  }

  paused = true;
  driftless::robot::subsystems::drivetrain::Velocity stopped{0, 0};
  setDriveVelocity(stopped);

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDTurn::resume() {
  if (m_mutex) {
    m_mutex->take();
  }

  paused = false;
  m_rotational_pid.reset();

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDTurn::turnToAngle(const std::shared_ptr<driftless::robot::Robot>& robot,
                          double velocity, double theta,
                          ETurnDirection direction) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_robot = robot;
  double drive_radius{getDriveRadius()};
  driftless::robot::subsystems::odometry::Position position{getPosition()};

  m_turn_direction = direction;
  m_max_velocity = velocity * drive_radius;

  double target_x = position.x + (TURN_TO_ANGLE_DISTANCE * std::cos(theta));
  double target_y = position.y + (TURN_TO_ANGLE_DISTANCE * std::sin(theta));
  m_target_point.setX(target_x);
  m_target_point.setY(target_y);

  target_reached = false;
  forced_direction_reached = false;

  m_rotational_pid.reset();

  paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDTurn::turnToPoint(const std::shared_ptr<driftless::robot::Robot>& robot,
                          double velocity, Point point,
                          ETurnDirection direction) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_robot = robot;
  double drive_radius{getDriveRadius()};

  m_turn_direction = direction;
  m_max_velocity = velocity * drive_radius;

  m_target_point = point;

  target_reached = false;
  forced_direction_reached = false;

  m_rotational_pid.reset();

  paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDTurn::targetReached() { return target_reached; }

void PIDTurn::setDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDTurn::setMutex(std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDTurn::setTask(std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDTurn::setRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDTurn::setTargetTolerance(double target_tolerance) {
  m_target_tolerance = target_tolerance;
}

void PIDTurn::setTargetVelocity(double target_velocity) {
  m_target_velocity = target_velocity;
}
}  // namespace motion
}  // namespace control
}  // namespace driftless