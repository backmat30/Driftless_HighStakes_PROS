#include "driftless/auton/BlueRushAuton.hpp"

#include "pros/screen.hpp"

namespace driftless {
namespace auton {
void BlueRushAuton::startColorSort() {
  m_process_system->sendCommand(
      processes::EProcess::AUTO_RING_REJECTION,
      processes::EProcessCommand::AUTO_RING_REJECTION_REJECT_RINGS, m_robot,
      m_alliance);
}
void BlueRushAuton::calibrateArm() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_CALIBRATE);
}

void BlueRushAuton::armGoNeutral() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
}

void BlueRushAuton::setClamp(bool clamped) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAMP,
                       robot::subsystems::ESubsystemCommand::CLAMP_SET_STATE,
                       clamped);
}

void BlueRushAuton::setElevatorVoltage(double voltage) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_SET_VOLTAGE, voltage);
}

void BlueRushAuton::waitForAllianceRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasAllianceRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void BlueRushAuton::waitForOpposingRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasOpposingRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void BlueRushAuton::setIntakeVoltage(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_VOLTAGE,
                       voltage);
}

void BlueRushAuton::setIntakeHeight(bool high) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_HEIGHT,
                       high);
}

void BlueRushAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemCommand::ODOMETRY_SET_POSITION, x, y, theta);
}

void BlueRushAuton::followPath(std::vector<control::Point>& path,
                               double velocity) {
  m_control_system->sendCommand(control::EControl::PATH_FOLLOWER,
                                control::EControlCommand::FOLLOW_PATH, &m_robot,
                                path, velocity);
}

void BlueRushAuton::setFollowPathVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::PATH_FOLLOWER,
      control::EControlCommand::PATH_FOLLOWER_SET_VELOCITY, velocity);
}

void BlueRushAuton::goToPoint(double x, double y, double velocity) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::GO_TO_POINT, &m_robot,
                                velocity, x, y);
}

void BlueRushAuton::setGoToPointVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::EControlCommand::GO_TO_POINT_SET_VELOCITY, velocity);
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
    m_delayer->delay(LOOP_DELAY);
  }
}

void BlueRushAuton::turnToPoint(double x, double y, double velocity,
                                control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_POINT,
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
    m_delayer->delay(LOOP_DELAY);
  }
}

void BlueRushAuton::turnToAngle(double theta, double velocity,
                                control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_ANGLE,
                                &m_robot, velocity, theta, direction);
}

void BlueRushAuton::waitForTurnToAngle(double theta, uint32_t timeout,
                                       double tolerance) {
  uint32_t current_time{m_clock->getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  while (!turnTargetReached() && current_time < end_time &&
         std::abs(bindRadians(current_position.theta - theta)) > tolerance) {
    current_time = m_clock->getTime();
    current_position = getOdomPosition();
    m_delayer->delay(LOOP_DELAY);
  }
}

void BlueRushAuton::driveStraight(double distance, double velocity,
                                  double theta) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::DRIVE_STRAIGHT,
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
    m_delayer->delay(LOOP_DELAY);
  }
}

void BlueRushAuton::delay(uint32_t delay_time) {
  uint32_t current_time{getTime()};
  uint32_t end_time{current_time + delay_time};

  while (current_time < end_time) {
    current_time = getTime();
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
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  return position;
}

double BlueRushAuton::getOdomVelocity() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  double velocity{
      std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2))};
  return velocity;
}

bool BlueRushAuton::followPathTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::PATH_FOLLOWER,
      control::EControlState::PATH_FOLLOWER_TARGET_REACHED))};
  return target_reached;
}

bool BlueRushAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POINT_TARGET_REACHED))};
  return target_reached;
}

bool BlueRushAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION, control::EControlState::TURN_TARGET_REACHED))};
  return target_reached;
}

bool BlueRushAuton::driveStraightTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::DRIVE_STRAIGHT_TARGET_REACHED))};
  return target_reached;
}

bool BlueRushAuton::hasAllianceRing() {
  bool has_alliance_ring{};

  void* has_ring_state{m_robot->getState(
      robot::subsystems::ESubsystem::RING_SORT,
      robot::subsystems::ESubsystemState::RING_SORT_HAS_RING)};
  bool has_ring{has_ring_state != nullptr &&
                *static_cast<bool*>(has_ring_state)};

  void* ring_rgb_state{
      m_robot->getState(robot::subsystems::ESubsystem::RING_SORT,
                        robot::subsystems::ESubsystemState::RING_SORT_GET_RGB)};
  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

  if (has_ring) {
    has_alliance_ring =
        ((m_alliance->getAlliance() == alliance::EAlliance::RED &&
          ring_rgb.red > ring_rgb.blue) ||
         (m_alliance->getAlliance() == alliance::EAlliance::BLUE &&
          ring_rgb.blue > ring_rgb.red));
  }

  return has_alliance_ring;
}

bool BlueRushAuton::hasOpposingRing() {
  bool has_opposing_ring{};

  void* has_ring_state{m_robot->getState(
      robot::subsystems::ESubsystem::RING_SORT,
      robot::subsystems::ESubsystemState::RING_SORT_HAS_RING)};
  bool has_ring{has_ring_state != nullptr &&
                *static_cast<bool*>(has_ring_state)};

  void* ring_rgb_state{
      m_robot->getState(robot::subsystems::ESubsystem::RING_SORT,
                        robot::subsystems::ESubsystemState::RING_SORT_GET_RGB)};
  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

  if (has_ring) {
    has_opposing_ring =
        ((m_alliance->getAlliance() == alliance::EAlliance::RED &&
          ring_rgb.red < ring_rgb.blue) ||
         (m_alliance->getAlliance() == alliance::EAlliance::BLUE &&
          ring_rgb.blue < ring_rgb.red));
  }

  return has_opposing_ring;
}

std::string BlueRushAuton::getName() { return AUTON_NAME; }

void BlueRushAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system,
    std::shared_ptr<driftless::processes::ProcessSystem>& process_system) {
  m_robot = robot;
  m_control_system = control_system;
  m_process_system = process_system;
}

void BlueRushAuton::run(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system,
    std::shared_ptr<driftless::processes::ProcessSystem>& process_system,
    std::shared_ptr<driftless::alliance::IAlliance>& alliance,
    std::shared_ptr<rtos::IClock>& clock,
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_clock = clock;
  m_delayer = std::move(delayer);
  m_control_system = control_system;
  m_process_system = process_system;
  m_robot = robot;
  m_alliance = alliance;

  // Set the robots starting values
  uint32_t start_time{getTime()};
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    setOdomPosition(109.0, 125.0, M_PI / 2.0);
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    setOdomPosition(144.0 - 109.0, 125.0, M_PI / 2.0);
  robot::subsystems::odometry::Position position{getOdomPosition()};
  double velocity{getOdomVelocity()};

  uint32_t current_time{start_time};
  control::Point target_point{};
  double target_distance{};
  double target_velocity{};
  double target_angular_velocity{1.75 * M_PI};

  startColorSort();

  // Start the rush path
  std::vector<control::Point> rush_control_points{};
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    rush_control_points = std::vector<control::Point>{
        control::Point{109.0, 125.0}, control::Point{108.0, 102.0},
        control::Point{113.5, 99.5}, control::Point{119.0, 83.0}};

  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    rush_control_points =
        std::vector<control::Point>{control::Point{144.0 - 109.0, 125.0},
                                    control::Point{144.0 - 108.0, 102.0},
                                    control::Point{144.0 - 113.0, 99.0},
                                    control::Point{144.0 - 120.0, 84.0}};
  }

  std::vector<control::Point> rush_path{
      control::path::BezierCurveInterpolation::calculate(rush_control_points)};
  target_point = rush_control_points.back();
  target_velocity = 72.0;

  followPath(rush_path, target_velocity);
  // Set up subsystems while moving to the path
  setIntakeVoltage(12.0);
  calibrateArm();
  m_delayer->delay(75);
  setIntakeVoltage(0.0);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 28.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }

  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 15.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  armGoNeutral();

  target_velocity = 16.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  position = getOdomPosition();
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);

  setClamp(true);
  m_control_system->pause();

  // raise the intake to get ready for the ring stacks
  setIntakeHeight(true);
  setIntakeVoltage(12.0);
  m_delayer->delay(50);

  // Go to first ring stack
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{124.0, 92.0};
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity,
                control::motion::ETurnDirection::CLOCKWISE);
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 123.0, 92.0};
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity,
                control::motion::ETurnDirection::COUNTERCLOCKWISE);
  }

  waitForTurnToPoint(target_point.getX(), target_point.getY(), 500,
                     M_PI / 10.0);
  setElevatorVoltage(12.0);
  delay(400);
  target_velocity = 32.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  while (target_distance > 6.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  setGoToPointVelocity(8.0);
  setElevatorVoltage(6.0);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 0.5);
  setIntakeHeight(false);
  setClamp(false);
  m_control_system->pause();

  // wait until the robot sees an alliance ring to continue the path
  waitForAllianceRing(500);
  position = getOdomPosition();
  target_velocity = 36.0;
  driveStraight(12.0, target_velocity, position.theta);
  waitForDriveStraight(12.0, 1000, 0.5);
  m_control_system->pause();
  if (hasAllianceRing()) {
    setIntakeVoltage(-12.0);
  }

  // go to the next mobile goal
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{84.0, 121.5};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 84.0, 121.5};

  position = getOdomPosition();
  turnToPoint(mirrorValue(target_point.getX(), position.x),
              mirrorValue(target_point.getY(), position.y),
              target_angular_velocity, control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(mirrorValue(target_point.getX(), position.x),
                     mirrorValue(target_point.getY(), position.y), 800,
                     M_PI / 20.0);
  setIntakeVoltage(0.0);
  setElevatorVoltage(0.0);
  target_velocity = 72.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  // delay until close to the goal
  while (target_distance > 14.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  // slow down near the target
  target_velocity = 16.0;
  setGoToPointVelocity(target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 0.5);
  setClamp(true);
  delay(75);

  // get orange preload
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{61.0, 118.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 61.0, 118.0};
  target_velocity = 72.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 24.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 32.0;
  setElevatorVoltage(0.0);
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity,
                control::motion::ETurnDirection::COUNTERCLOCKWISE);
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity,
                control::motion::ETurnDirection::CLOCKWISE);

  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                     M_PI / 4.0);
  setElevatorVoltage(12.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  setIntakeVoltage(12.0);
  setElevatorVoltage(12.0);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 1.0);
  waitForAllianceRing(3000);

  // move towards next rings
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{32.0, 101.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 32.0, 101.0};
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  setIntakeVoltage(12.0);
  setElevatorVoltage(12.0);

  while (target_distance > 20.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  setIntakeHeight(true);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);
  setIntakeHeight(false);

  waitForOpposingRing(400);
  position = getOdomPosition();
  driveStraight(15.0, target_velocity, position.theta);
  waitForDriveStraight(15.0, 1000, 0.5);
  m_control_system->pause();
  waitForOpposingRing(1000);

  // move to next rings (cont.)

  setIntakeHeight(true);
  setElevatorVoltage(0.0);
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{30.5, 116.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 30.5, 116.0};
  target_velocity = 32.0;
  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 15.0);
  setElevatorVoltage(12.0);
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);
  setIntakeHeight(false);

  waitForAllianceRing(400);
  position = getOdomPosition();
  target_velocity = 40.0;
  driveStraight(10.0, target_velocity, position.theta);
  waitForDriveStraight(10.0, 1000, 0.5);
  m_control_system->pause();
  waitForAllianceRing(2000);
  setElevatorVoltage(0.0);
  delay(500);

  // Line up for corner
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{45.0, 130.0};
    target_velocity = 16.0;
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity, control::motion::ETurnDirection::AUTO);
    waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                       M_PI / 25.0);
    setElevatorVoltage(12.0);

    goToPoint(target_point.getX(), target_point.getY(), target_velocity);
    waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 0.5);
    position = getOdomPosition();
    target_velocity = 42.0;
    driveStraight(5.0, target_velocity, position.theta);
    waitForDriveStraight(5.0, 700, 0.5);
    m_control_system->pause();
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    setElevatorVoltage(12.0);
    
    position = getOdomPosition();
    driveStraight(8.0, target_velocity, position.theta);
    waitForDriveStraight(8.0, 700, 0.5);
    m_control_system->pause();
  }

  // attempt to clear corner :)
  setIntakeHeight(true);

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{0.0, 140.0};
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity,
                control::motion::ETurnDirection::COUNTERCLOCKWISE);
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 0.0, 140.0};
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity,
                control::motion::ETurnDirection::CLOCKWISE);
  }
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 3000,
                     M_PI / 25.0);

  target_velocity = 36.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1750, 0.5);

  setIntakeHeight(false);

  target_velocity = 20;
  position = getOdomPosition();
  while (current_time < start_time + 23500) {
    driveStraight(5.0, target_velocity, position.theta);
    waitForDriveStraight(5.0, 500, 0.5);
    m_control_system->pause();
    delay(100);
    driveStraight(-4.5, target_velocity, position.theta);
    waitForDriveStraight(-4.5, 500, 0.5);
    m_control_system->pause();
    delay(100);
    current_time = getTime();
  }
  target_velocity = 48.0;
  driveStraight(-10.0, target_velocity, position.theta);
  waitForDriveStraight(-10.0, 1000, 0.5);
  driveStraight(15.0, target_velocity, position.theta);
  waitForDriveStraight(15.0, 1000, 0.5);
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    driveStraight(-10.0, target_velocity, 3.0 * M_PI / 4.0);
    waitForDriveStraight(-25.0, 2000, 0.5);
    turnToAngle(-M_PI / 4.0, target_angular_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(-M_PI / 4.0, 1000, M_PI / 25.0);
    setClamp(false);
    driveStraight(-10.0, target_velocity, -M_PI / 4.0);
    waitForDriveStraight(-10.0, 500, 0.5);
    driveStraight(25.0, target_velocity, -M_PI / 4.0);
    waitForDriveStraight(25.0, 1000, 0.5);
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    driveStraight(-10.0, target_velocity, M_PI / 4.0);
    waitForDriveStraight(-25.0, 2000, 0.5);
    turnToAngle(-3.0 * M_PI / 4.0, target_angular_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(-3.0 * M_PI / 4.0, 1000, M_PI / 25.0);
    setClamp(false);
    driveStraight(-10.0, target_velocity, -3.0 * M_PI / 4.0);
    waitForDriveStraight(-10.0, 500, 0.5);
    driveStraight(25.0, target_velocity, -3.0 * M_PI / 4.0);
    waitForDriveStraight(25.0, 1000, 0.5);
  }
  m_control_system->pause();
  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // Print the run-time for routing purposes, determine how much more can be
  // done after current tasks
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 5, "runtime: %7.2f",
                      ((m_clock->getTime() - start_time) / 1000.0));
}
}  // namespace auton
}  // namespace driftless