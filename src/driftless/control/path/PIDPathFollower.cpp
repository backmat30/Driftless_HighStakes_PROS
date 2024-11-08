#include "driftless/control/path/PIDPathFollower.hpp"

namespace driftless {
namespace control {
namespace path {
void PIDPathFollower::taskLoop(void* params) {
  PIDPathFollower* instance{static_cast<PIDPathFollower*>(params)};
  while (true) {
    instance->taskUpdate();
  }
}

void PIDPathFollower::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!paused && !target_reached) {
    driftless::robot::subsystems::odometry::Position position{getPosition()};
    double distance_to_target{calculateDistanceToTarget(position)};
    double velocity{distance(0, 0, position.xV, position.yV)};
    if (found_index == m_control_path.size() - 1 &&
        distance_to_target < m_target_tolerance &&
        velocity < m_target_velocity) {
      target_reached = true;
      robot::subsystems::drivetrain::Velocity stop{0, 0};
      setDriveVelocity(stop);
    } else {
      updateFoundPoints(position);
      Point follow_point{calculateFollowPoint(position)};
      updateVelocity(position, follow_point);
    }
  }

  if (m_mutex) {
    m_mutex->give();
  }

  m_delayer->delay(TASK_DELAY);
}

void PIDPathFollower::setDriveVelocity(
    robot::subsystems::drivetrain::Velocity velocity) {
  if (m_robot) {
    m_robot->sendCommand(DRIVE_SUBSYSTEM_NAME, DRIVE_SET_VELOCITY_COMMAND_NAME,
                         velocity);
  }
}

double PIDPathFollower::getDriveRadius() {
  double radius{};
  if (m_robot) {
    radius = *(static_cast<double*>(m_robot->getState(
        DRIVE_SUBSYSTEM_NAME, DRIVE_GET_RADIUS_COMMAND_NAME)));
  }
  return radius;
}

robot::subsystems::odometry::Position PIDPathFollower::getPosition() {
  robot::subsystems::odometry::Position position{};

  if (m_robot) {
    position =
        *(static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
            ODOMETRY_SUBSYSTEM_NAME, ODOMETRY_GET_POSITION_COMMAND_NAME)));
  }

  return position;
}

double PIDPathFollower::calculateDistanceToTarget(
    driftless::robot::subsystems::odometry::Position position) {
  double target_distance{};
  if (!m_control_path.empty()) {
    Point end_point{m_control_path.back()};
    target_distance =
        distance(position.x, position.y, end_point.getX(), end_point.getY());
  }
  return target_distance;
}

void PIDPathFollower::updateFoundPoints(
    robot::subsystems::odometry::Position position) {
  bool continue_loop{true};
  while (continue_loop && found_index < m_control_path.size() - 1) {
    continue_loop = false;
    Point next_point{m_control_path[found_index + 1]};
    double next_distance{
        distance(position.x, position.y, next_point.getX(), next_point.getY())};
    if (next_distance <= m_follow_distance) {
      ++found_index;
      continue_loop = true;
    }
  }
}

Point PIDPathFollower::calculateFollowPoint(
    driftless::robot::subsystems::odometry::Position position) {
  Point follow_point{};
  if (!m_control_path.empty()) {
    if (found_index >= m_control_path.size() - 1) {
      // go to the last point if you have already hit every point
      follow_point = m_control_path.back();
    } else {
      // Get the relevant position data

      // the last point reached (found) in the route
      Point p1{m_control_path[found_index]};
      // the next point to route to
      Point p2{m_control_path[found_index + 1]};
      // x offset of p1
      double x1{p1.getX() - position.x};
      // y offset of p1
      double y1{p1.getY() - position.y};
      // x offset p2
      double x2{p2.getX() - position.x};
      // y offset p2
      double y2{p2.getY() - position.y};

      // Calculate distances
      // x distance
      double dx{x2 - x1};
      // y distance
      double dy{y2 - y1};
      // total distance between p1 and p2
      double dr{distance(x1, y1, x2, y2)};
      // determinant of the 2x2 matrix made by p1 and p2:
      //  x1  y1
      //  x2  y2
      double D{(x1 * y2) - (x2 * y1)};
      // find the discriminant to determine if there are intersections
      //  discriminant < 0     means no intersections
      //  discriminant = 0     means 1 intersection
      //  discriminant > 0     means 2 intersections
      double discriminant{(std::pow(m_follow_distance, 2) * std::pow(dr, 2)) -
                          std::pow(D, 2)};

      // If there are no intersections, return the next target point
      if (discriminant < 0) {
        follow_point = p2;
      } else {
        // Find the intersection points
        double ddy{D * dy};
        double nddx{-1 * D * dx};
        // sign (- or +) of y
        int8_t sdy{sign(dy)};
        double ady{std::abs(dy)};
        double sqrt_discriminant{std::sqrt(discriminant)};
        double dr2{std::pow(dr, 2)};
        // intersection solutions
        double solution1X{(ddy + (sdy * dx * sqrt_discriminant)) / dr2};
        double solution1Y{(nddx + (ady * sqrt_discriminant)) / dr2};
        double solution2X{(ddy - (sdy * dx * sqrt_discriminant)) / dr2};
        double solution2Y{(nddx - (ady * sqrt_discriminant)) / dr2};
        Point solution1{solution1X + position.x, solution1Y + position.y};
        Point solution2{solution2X + position.x, solution2Y + position.y};

        // Pick the point which is further along the line
        double distance1{
            distance(solution1.getX(), solution1.getY(), p2.getX(), p2.getY())};
        double distance2{
            distance(solution2.getX(), solution2.getY(), p2.getX(), p2.getY())};
        if (distance1 < distance2)
          follow_point = solution1;
        else
          follow_point = solution2;

        // Determine if the point is on the bounded line segment
        double sx{follow_point.getX() - position.x};
        double sy{follow_point.getY() - position.y};
        if (sx < std::min(x1, x2) || sx > std::max(x1, x2) ||
            sy < std::min(y1, y2) || sy > std::max(y1, y2)) {
          follow_point = p1;
        }
      }
    }
  }
  return follow_point;
}

void PIDPathFollower::updateVelocity(
    driftless::robot::subsystems::odometry::Position position,
    Point follow_point) {
  // difference between target and current position
  double x_error{follow_point.getX() - position.x};
  double y_error{follow_point.getY() - position.y};

  // difference in angle between current direction and the direction the target
  // is in
  double rotational_error{
      bindRadians(std::atan2(y_error, x_error) - position.theta)};
  // length of the vector perpendicular to the robot direction that intersects
  // with the target position
  double linear_error{std::cos(rotational_error) *
                      std::sqrt(std::pow(x_error, 2) + std::pow(y_error, 2))};
  // if the linear error is negative, reverse the rotational error
  if (linear_error < 0) rotational_error = bindRadians(rotational_error + M_PI);
  // get the control value from the pid
  double linear_control{m_linear_pid.getControlValue(0, linear_error)};
  // if the control value is out of the bounds of the max velocity, set it to
  // the (+/-)max velocity
  if (std::abs(linear_control) > m_max_velocity)
    linear_control *= m_max_velocity / std::abs(linear_control);

  // get the control from PID
  double rotational_control{};
  // if the robot has to turn more than 45 degrees, use PID
  // otherwise, use the linear control to find the control value
  if (std::abs(rotational_error) > M_PI / 4)
    rotational_control = m_rotational_pid.getControlValue(0, rotational_error);
  else
    rotational_control = (2 * getDriveRadius() * std::sin(rotational_error) /
                          m_follow_distance) *
                         std::abs(linear_control);

  // define the velocity for each side of the robot and pass into velocity
  // struct
  double left_velocity{linear_control - rotational_control};
  double right_velocity{linear_control + rotational_control};
  robot::subsystems::drivetrain::Velocity velocity{left_velocity,
                                                   right_velocity};
  // pros::screen::print(pros::E_TEXT_LARGE_CENTER, 9, "%7.2f, %7.2f",
  // left_velocity, right_velocity);
  setDriveVelocity(velocity);
}

void PIDPathFollower::init() {
  m_linear_pid.reset();
  m_rotational_pid.reset();
}

void PIDPathFollower::run() {
  if (m_task) {
    m_task->start(PIDPathFollower::taskLoop, this);
  }
}

void PIDPathFollower::followPath(const std::shared_ptr<robot::Robot>& robot,
                                 const std::vector<Point>& control_path,
                                 double velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_robot = robot;
  m_control_path = control_path;
  found_index = 0;
  m_max_velocity = velocity;
  target_reached = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDPathFollower::setVelocity(double velocity) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_max_velocity = velocity;
  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDPathFollower::pause() {
  if (m_mutex) {
    m_mutex->take();
  }

  paused = true;
  robot::subsystems::drivetrain::Velocity stop{0, 0};
  setDriveVelocity(stop);

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDPathFollower::resume() {
  if (m_mutex) {
    m_mutex->take();
  }
  paused = false;
  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDPathFollower::targetReached() { return target_reached; }

void PIDPathFollower::setDelayer(
    const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDPathFollower::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDPathFollower::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDPathFollower::setLinearPID(PID linear_pid) {
  m_linear_pid = linear_pid;
}

void PIDPathFollower::setRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDPathFollower::setFollowDistance(double follow_distance) {
  m_follow_distance = follow_distance;
}

void PIDPathFollower::setTargetTolerance(double target_tolerance) {
  m_target_tolerance = target_tolerance;
}

void PIDPathFollower::setTargetVelocity(double target_velocity) {
  m_target_velocity = target_velocity;
}
}  // namespace path
}  // namespace control
}  // namespace driftless