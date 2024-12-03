#include "driftless/auton/BlueRushAuton.hpp"

#include "pros/screen.hpp"

namespace driftless {
namespace auton {
void BlueRushAuton::calibrateArm() {
  m_robot->sendCommand(ARM_SUBSYSTEM_NAME, ARM_CALIBRATE_COMMAND);
}

void BlueRushAuton::armGoNeutral() {
  m_robot->sendCommand(ARM_SUBSYSTEM_NAME, ARM_GO_NEUTRAL_COMMAND);
}

void BlueRushAuton::setClamp(bool clamped) {
  m_robot->sendCommand(CLAMP_SUBSYSTEM_NAME, CLAMP_SET_STATE_COMMAND, clamped);
}

void BlueRushAuton::setElevatorVoltage(double voltage) {
  m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME, ELEVATOR_SET_VOLTAGE, voltage);
}

void BlueRushAuton::updateRingSort() {
  void* position_state{
      m_robot->getState(ELEVATOR_SUBSYSTEM_NAME, ELEVATOR_POSITION_STATE)};
  double position{*static_cast<double*>(position_state)};

  void* distance_to_end_state{m_robot->getState(
      RING_SORT_SUBSYSTEM_NAME, RING_SORT_GET_DISTANCE_TO_END_STATE)};
  double distance_to_end{*static_cast<double*>(distance_to_end_state)};

  if (hasOpposingRing()) {
    ring_sort_latest_ring_pos = position;
  }

  if (position <= ring_sort_latest_ring_pos + distance_to_end &&
      position >= ring_sort_latest_ring_pos - distance_to_end) {
    m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME,
                         ELEVATOR_DEPLOY_REJECTOR_COMMAND);
  } else {
    m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME,
                         ELEVATOR_RETRACT_REJECTOR_COMMAND);
  }
}

void BlueRushAuton::waitForAllianceRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasAllianceRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
    updateRingSort();
  }
}

void BlueRushAuton::waitForOpposingRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasOpposingRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
    updateRingSort();
  }
}

void BlueRushAuton::spinIntake(double voltage) {
  m_robot->sendCommand(INTAKE_SUBSYSTEM_NAME, INTAKE_SPIN_COMMAND, voltage);
}

void BlueRushAuton::setIntakeHeight(bool high) {
  m_robot->sendCommand(INTAKE_SUBSYSTEM_NAME, INTAKE_SET_HEIGHT_COMMAND, high);
}

void BlueRushAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(ODOMETRY_SUBSYSTEM_NAME, ODOMETRY_SET_POSITION_COMMAND,
                       x, y, theta);
}

void BlueRushAuton::followPath(std::vector<control::Point>& path,
                               double velocity) {
  m_control_system->sendCommand(PATH_FOLLOWER_CONTROL_NAME, FOLLOW_PATH_COMMAND,
                                &m_robot, path, velocity);
}

void BlueRushAuton::setFollowPathVelocity(double velocity) {
  m_control_system->sendCommand(PATH_FOLLOWER_CONTROL_NAME,
                                SET_PATH_FOLLOWER_VELOCTY_COMMAND, velocity);
}

void BlueRushAuton::goToPoint(double x, double y, double velocity) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME, GO_TO_POINT_COMMAND,
                                &m_robot, velocity, x, y);
}

void BlueRushAuton::setGoToPointVelocity(double velocity) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME,
                                SET_GO_TO_POINT_VELOCITY_COMMAND, velocity);
}

void BlueRushAuton::waitForGoToPoint(double target_x, double target_y,
                                     uint32_t timeout, double tolerance) {
  uint32_t current_time{m_clock->getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double distance_to_target{
      distance(current_position.x, current_position.y, target_x, target_y)};
  while (!goToPointTargetReached() && current_time < end_time &&
         std::abs(distance_to_target) > tolerance) {
    current_time = m_clock->getTime();
    current_position = getOdomPosition();
    distance_to_target =
        distance(current_position.x, current_position.y, target_x, target_y);
    updateRingSort();
    m_delayer->delay(LOOP_DELAY);
  }
}

void BlueRushAuton::turnToPoint(double x, double y, double velocity,
                                control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME, TURN_TO_POINT_COMMAND,
                                &m_robot, velocity, x, y, direction);
}

void BlueRushAuton::waitForTurnToPoint(double x, double y, uint32_t timeout,
                                       double tolerance) {
  uint32_t current_time{m_clock->getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double angle_difference{
      bindRadians(angle(current_position.x, current_position.y, x, y) -
                  current_position.theta)};
  while (!turnTargetReached() && current_time < end_time &&
         std::abs(angle_difference) > tolerance) {
    current_time = m_clock->getTime();
    current_position = getOdomPosition();
    angle_difference =
        bindRadians(angle(current_position.x, current_position.y, x, y) -
                    current_position.theta);
    updateRingSort();
    m_delayer->delay(LOOP_DELAY);
  }
}

void BlueRushAuton::turnToAngle(double theta, double velocity,
                                control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME, TURN_TO_ANGLE_COMMAND,
                                &m_robot, velocity, theta, direction);
}

void BlueRushAuton::waitForTurnToAngle(double theta, uint32_t timeout,
                                       double tolerance) {
  uint32_t current_time{m_clock->getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  while (!turnTargetReached() && current_time < end_time &&
         std::abs(current_position.theta - theta) > tolerance) {
    current_time = m_clock->getTime();
    current_position = getOdomPosition();
    updateRingSort();
    m_delayer->delay(LOOP_DELAY);
  }
}

void BlueRushAuton::driveStraight(double distance, double velocity,
                                  double theta) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME, DRIVE_STRAIGHT_COMMAND,
                                &m_robot, velocity, distance, theta);
}

void BlueRushAuton::waitForDriveStraight(double target_distance,
                                         uint32_t timeout, double tolerance) {
  uint32_t current_time{getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position start_position{getOdomPosition()};
  robot::subsystems::odometry::Position current_position{start_position};
  double current_distance{distance(start_position.x, start_position.y,
                                   current_position.x, current_position.y)};

  while (!driveStraightTargetReached() && current_time < end_time &&
         std::abs(current_distance - target_distance) > tolerance) {
    current_time = getTime();
    current_position = getOdomPosition();
    current_distance = distance(start_position.x, start_position.y,
                                current_position.x, current_position.y);
    updateRingSort();
    m_delayer->delay(LOOP_DELAY);
  }
}

void BlueRushAuton::delay(uint32_t delay_time) {
  uint32_t current_time{getTime()};
  uint32_t end_time{current_time + delay_time};

  while (current_time < end_time) {
    current_time = getTime();
    updateRingSort();
    m_delayer->delay(LOOP_DELAY);
  }
}

uint32_t BlueRushAuton::getTime() {
  uint32_t current_time{};
  if (m_clock) {
    current_time = m_clock->getTime();
  }
  return current_time;
}

robot::subsystems::odometry::Position BlueRushAuton::getOdomPosition() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          ODOMETRY_SUBSYSTEM_NAME, ODOMETRY_GET_POSITION_STATE))};

  return position;
}

double BlueRushAuton::getOdomVelocity() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          ODOMETRY_SUBSYSTEM_NAME, ODOMETRY_GET_POSITION_STATE))};

  double velocity{
      std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2))};
  return velocity;
}

bool BlueRushAuton::followPathTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      PATH_FOLLOWER_CONTROL_NAME, PATH_FOLLOWER_TARGET_REACHED_STATE))};
  return target_reached;
}

bool BlueRushAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      MOTION_CONTROL_NAME, GO_TO_POINT_TARGET_REACHED_STATE))};
  return target_reached;
}

bool BlueRushAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      MOTION_CONTROL_NAME, TURN_TARGET_REACHED_STATE))};
  return target_reached;
}

bool BlueRushAuton::driveStraightTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      MOTION_CONTROL_NAME, DRIVE_STRAIGHT_TARGET_REACHED_STATE))};
  return target_reached;
}

bool BlueRushAuton::hasAllianceRing() {
  bool has_alliance_ring{};

  void* has_ring_state{
      m_robot->getState(RING_SORT_SUBSYSTEM_NAME, RING_SORT_HAS_RING_STATE)};
  bool has_ring{has_ring_state != nullptr &&
                *static_cast<bool*>(has_ring_state)};

  void* ring_rgb_state{
      m_robot->getState(RING_SORT_SUBSYSTEM_NAME, RING_SORT_GET_RGB_STATE)};
  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

  if (has_ring) {
    has_alliance_ring =
        ((m_alliance->getName() == "RED" && ring_rgb.red > ring_rgb.blue) ||
         (m_alliance->getName() == "BLUE" && ring_rgb.blue > ring_rgb.red));
  }

  return has_alliance_ring;
}

bool BlueRushAuton::hasOpposingRing() {
  bool has_opposing_ring{};

  void* has_ring_state{
      m_robot->getState(RING_SORT_SUBSYSTEM_NAME, RING_SORT_HAS_RING_STATE)};
  bool has_ring{has_ring_state != nullptr &&
                *static_cast<bool*>(has_ring_state)};

  void* ring_rgb_state{
      m_robot->getState(RING_SORT_SUBSYSTEM_NAME, RING_SORT_GET_RGB_STATE)};
  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

  if (has_ring) {
    has_opposing_ring =
        ((m_alliance->getName() == "RED" && ring_rgb.red < ring_rgb.blue) ||
         (m_alliance->getName() == "BLUE" && ring_rgb.blue < ring_rgb.red));
  }

  return has_opposing_ring;
}

std::string BlueRushAuton::getName() { return AUTON_NAME; }

void BlueRushAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void BlueRushAuton::run(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system,
    std::shared_ptr<driftless::alliance::IAlliance>& alliance,
    std::shared_ptr<rtos::IClock>& clock,
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_clock = clock;
  m_delayer = std::move(delayer);
  m_control_system = control_system;
  m_robot = robot;
  m_alliance = alliance;

  // Set the robots starting values
  uint32_t start_time{getTime()};
  setOdomPosition(109.0, 125.0, M_PI / 2.0);
  robot::subsystems::odometry::Position position{getOdomPosition()};
  double velocity{getOdomVelocity()};

  uint32_t current_time{start_time};
  control::Point target_point{};
  double target_distance{};
  double target_velocity{};

  // Start the rush path
  std::vector<control::Point> rush_control_points{
      control::Point{109.0, 115.0}, control::Point{112.0, 97.0},
      control::Point{112.5, 96.0}, control::Point{120.0, 84.85}};
  std::vector<control::Point> rush_path{
      control::path::BezierCurveInterpolation::calculate(rush_control_points)};
  target_point = rush_control_points.back();

  target_velocity = 48.0;

  followPath(rush_path, target_velocity);
  // Set up subsystems while moving to the path
  spinIntake(12.0);
  calibrateArm();
  m_delayer->delay(50);
  spinIntake(0.0);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 30.5) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 48;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  armGoNeutral();
  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 15.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  setGoToPointVelocity(12.0);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);

  setClamp(true);
  m_control_system->pause();

  // raise the intake to get ready for the ring stacks
  setIntakeHeight(true);
  spinIntake(12.0);
  m_delayer->delay(50);

  // Go to first ring stack
  target_point = control::Point{123.5, 97.75};
  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::CLOCKWISE);
  waitForTurnToPoint(target_point.getX() - 3.0, target_point.getY(), 500,
                     M_PI / 10.0);
  setElevatorVoltage(12.0);
  delay(400);
  target_velocity = 16.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  while (target_distance > 6.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  setGoToPointVelocity(8.0);
  setIntakeHeight(false);
  setElevatorVoltage(6.0);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 0.5);
  setClamp(false);
  m_control_system->pause();

  // wait until the robot sees an alliance ring to continue the path
  waitForAllianceRing(500);
  if (!hasAllianceRing()) {
    position = getOdomPosition();
    target_velocity = 24.0;
    driveStraight(15.0, target_velocity, position.theta);
    waitForDriveStraight(15.0, 1000, 0.5);
    m_control_system->pause();
    waitForAllianceRing(3000);
  }

  setElevatorVoltage(0.0);
  spinIntake(-12.0);

  // go to the next mobile goal
  target_point = control::Point{82.0, 121.5};
  target_velocity = 12.0;
  position = getOdomPosition();
  turnToPoint(mirrorValue(target_point.getX(), position.x),
              mirrorValue(target_point.getY(), position.y), target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(mirrorValue(target_point.getX(), position.x),
                     mirrorValue(target_point.getY(), position.y), 500,
                     M_PI / 25.0);
  spinIntake(0.0);
  target_velocity = 48.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  // delay until close to the goal
  while (target_distance > 18.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  // slow down near the target
  target_velocity = 10.0;
  setGoToPointVelocity(target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 0.5);
  setClamp(true);
  setElevatorVoltage(12.0);

  // move towards next rings
  target_point = control::Point{44.0, 99.5};
  target_velocity = 48.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 32.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 16.0;
  setElevatorVoltage(0.0);
  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::COUNTERCLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                     M_PI / 4.0);
  setElevatorVoltage(12.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  setIntakeHeight(true);
  spinIntake(12.0);
  setElevatorVoltage(12.0);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);
  setIntakeHeight(false);

  waitForOpposingRing(400);
  position = getOdomPosition();
  driveStraight(15.0, target_velocity, position.theta);
  waitForDriveStraight(15.0, 1000, 0.5);
  m_control_system->pause();
  waitForOpposingRing(3000);

  // move to next rings (cont.)

  setIntakeHeight(true);
  setElevatorVoltage(0.0);
  target_point = control::Point{38.5, 110.0};
  target_velocity = 10.0;
  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 15.0);
  setElevatorVoltage(12.0);
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);
  setIntakeHeight(false);

  waitForAllianceRing(400);
  position = getOdomPosition();
  target_velocity = 24.0;
  driveStraight(15.0, target_velocity, position.theta);
  waitForDriveStraight(15.0, 1000, 0.5);
  m_control_system->pause();
  waitForAllianceRing(5000);

  // Go to orange preload
  target_point = control::Point{45.0, 130.0};
  target_velocity = 10.0;
  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                     M_PI / 25.0);
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 0.5);
  position = getOdomPosition();
  target_velocity = 24.0;
  driveStraight(10.0, target_velocity, position.theta);
  waitForDriveStraight(10.0, 700, 0.5);
  m_control_system->pause();
  waitForAllianceRing(1000);

  // attempt to clear corner :)
  target_point = control::Point{45.0, 125.0};
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 750, 0.5);
  
  setIntakeHeight(true);

  target_point = control::Point{0.0, 140.0};
  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::COUNTERCLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 3000,
                     M_PI / 25.0);

  target_velocity = 16.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1750, 0.5);

  setIntakeHeight(false);

  target_velocity = 10.0;
  while (current_time < start_time + 25000) {
    driveStraight(2.0, target_velocity, M_PI);
    waitForDriveStraight(2.0, 500, 0.5);
    m_control_system->pause();
    delay(100);
    driveStraight(-2.0, target_velocity, M_PI);
    waitForDriveStraight(-2.0, 500, 0.5);
    m_control_system->pause();
    delay(100);
    current_time = getTime();
  }
  driveStraight(-10.0, target_velocity, M_PI);
  waitForDriveStraight(-10.0, 1000, 0.5);
  driveStraight(15.0, target_velocity, M_PI);
  waitForDriveStraight(15.0, 1000, 0.5);
  driveStraight(-25.0, target_velocity, 3.0 * M_PI / 4.0);
  waitForDriveStraight(-25.0, 2000, 0.5);
  turnToAngle(-M_PI / 4.0, target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(-M_PI / 4.0, 1000, M_PI / 25.0);
  m_control_system->pause();
  setElevatorVoltage(0.0);
  spinIntake(0.0);

  // Print the run-time for routing purposes, determine how much more can be
  // done after current tasks
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 5, "runtime: %7.2f",
                      ((m_clock->getTime() - start_time) / 1000.0));
}
}  // namespace auton
}  // namespace driftless