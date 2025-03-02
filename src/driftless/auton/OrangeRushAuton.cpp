#include "driftless/auton/OrangeRushAuton.hpp"

#include "pros/screen.hpp"

namespace driftless {
namespace auton {
void OrangeRushAuton::startColorSort() {
  m_process_system->sendCommand(
      processes::EProcess::AUTO_RING_REJECTION,
      processes::EProcessCommand::AUTO_RING_REJECTION_REJECT_RINGS, &m_robot,
      m_alliance);
}
void OrangeRushAuton::calibrateArm() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_CALIBRATE);
}

void OrangeRushAuton::armGoNeutral() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
}

void OrangeRushAuton::armGoLoad() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
}

void OrangeRushAuton::armGoAllianceStake() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemCommand::ARM_GO_ALLIANCE_STAKE);
}

void OrangeRushAuton::setClamp(bool clamped) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAMP,
                       robot::subsystems::ESubsystemCommand::CLAMP_SET_STATE,
                       clamped);
}

void OrangeRushAuton::setElevatorVoltage(double voltage) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_SET_VOLTAGE, voltage);
}

void OrangeRushAuton::waitForAllianceRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasAllianceRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void OrangeRushAuton::waitForOpposingRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasOpposingRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void OrangeRushAuton::setIntakeVoltage(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_VOLTAGE,
                       voltage);
}

void OrangeRushAuton::setIntakeHeight(bool high) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_HEIGHT,
                       high);
}

void OrangeRushAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemCommand::ODOMETRY_SET_POSITION, x, y, theta);
}

void OrangeRushAuton::followPath(std::vector<control::Point>& path,
                                 double velocity) {
  m_control_system->sendCommand(control::EControl::PATH_FOLLOWER,
                                control::EControlCommand::FOLLOW_PATH, &m_robot,
                                path, velocity);
}

void OrangeRushAuton::setFollowPathVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::PATH_FOLLOWER,
      control::EControlCommand::PATH_FOLLOWER_SET_VELOCITY, velocity);
}

void OrangeRushAuton::goToPoint(double x, double y, double velocity) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::GO_TO_POINT, &m_robot,
                                velocity, x, y);
}

void OrangeRushAuton::setGoToPointVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::EControlCommand::GO_TO_POINT_SET_VELOCITY, velocity);
}

void OrangeRushAuton::waitForGoToPoint(double target_x, double target_y,
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

void OrangeRushAuton::turnToPoint(double x, double y, double velocity,
                                  control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_POINT,
                                &m_robot, velocity, x, y, direction);
}

void OrangeRushAuton::waitForTurnToPoint(double x, double y, uint32_t timeout,
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

void OrangeRushAuton::turnToAngle(double theta, double velocity,
                                  control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_ANGLE,
                                &m_robot, velocity, theta, direction);
}

void OrangeRushAuton::waitForTurnToAngle(double theta, uint32_t timeout,
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

void OrangeRushAuton::driveStraight(double distance, double velocity,
                                    double theta) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::DRIVE_STRAIGHT,
                                &m_robot, velocity, distance, theta);
}

void OrangeRushAuton::waitForDriveStraight(double target_distance,
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

void OrangeRushAuton::delay(uint32_t delay_time) {
  uint32_t current_time{getTime()};
  uint32_t end_time{current_time + delay_time};

  while (current_time < end_time) {
    current_time = getTime();
    m_delayer->delay(LOOP_DELAY);
  }
}

uint32_t OrangeRushAuton::getTime() {
  uint32_t current_time{};
  if (m_clock) {
    current_time = m_clock->getTime();
  }
  return current_time;
}

robot::subsystems::odometry::Position OrangeRushAuton::getOdomPosition() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  return position;
}

double OrangeRushAuton::getOdomVelocity() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  double velocity{
      std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2))};
  return velocity;
}

bool OrangeRushAuton::followPathTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::PATH_FOLLOWER,
      control::EControlState::PATH_FOLLOWER_TARGET_REACHED))};
  return target_reached;
}

bool OrangeRushAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POINT_TARGET_REACHED))};
  return target_reached;
}

bool OrangeRushAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION, control::EControlState::TURN_TARGET_REACHED))};
  return target_reached;
}

bool OrangeRushAuton::driveStraightTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::DRIVE_STRAIGHT_TARGET_REACHED))};
  return target_reached;
}

bool OrangeRushAuton::hasAllianceRing() {
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

bool OrangeRushAuton::hasOpposingRing() {
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

std::string OrangeRushAuton::getName() { return AUTON_NAME; }

void OrangeRushAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system,
    std::shared_ptr<driftless::processes::ProcessSystem>& process_system) {
  m_robot = robot;
  m_control_system = control_system;
  m_process_system = process_system;
}

void OrangeRushAuton::run(
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
    setOdomPosition(35.0, 114.5, M_PI / 2.0);
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    setOdomPosition(144.0 - 35.0, 114.5, M_PI / 2.0);
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
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    rush_control_points = std::vector<control::Point>{
        control::Point{35.0, 114.5}, control::Point{31.0, 97.0},
        control::Point{29.0, 96.0}, control::Point{24.0, 79.5}};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    rush_control_points = std::vector<control::Point>{
        control::Point{144.0 - 35.0, 114.5}, control::Point{144.0 - 32.0, 97.0},
        control::Point{144.0 - 31.5, 96.0},
        control::Point{144.0 - 23.75, 81.75}};

  std::vector<control::Point> rush_path{
      control::path::BezierCurveInterpolation::calculate(rush_control_points)};
  target_point = rush_control_points.back();

  target_velocity = 72.0;

  followPath(rush_path, target_velocity);
  // Set up subsystems while moving to the path
  calibrateArm();

  delay(75);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 14.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 16.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);

  setClamp(true);
  m_control_system->pause();
  delay(100);
  armGoNeutral();

  // Set up the path under the ladder
  std::vector<control::Point> under_ladder_control_points{};
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    under_ladder_control_points = std::vector<control::Point>{
        control::Point{24.0, 82.25}, control::Point{46.0, 95.0},
        control::Point{72.0, 70.0}, control::Point{90.0, 97.0}};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    under_ladder_control_points = std::vector<control::Point>{
        control::Point{144.0 - 24.0, 82.25}, control::Point{144.0 - 46.0, 97.0},
        control::Point{144.0 - 72.0, 70.0}, control::Point{144.0 - 88.0, 96.0}};

  std::vector<control::Point> under_ladder_path{
      control::path::BezierCurveInterpolation::calculate(
          under_ladder_control_points)};

  // follow the path under the ladder
  target_point = under_ladder_control_points.back();
  target_velocity = 12.0;
  position = getOdomPosition();

  followPath(under_ladder_path, target_velocity);

  setIntakeVoltage(12.0);
  setElevatorVoltage(12.0);
  delay(100);
  target_velocity = 48.0;
  setFollowPathVelocity(target_velocity);

  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  bool elevator_running{true};
  while (target_distance > 16.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
    if (elevator_running && hasOpposingRing()) {
      setElevatorVoltage(0.0);
      setIntakeVoltage(0.0);
      elevator_running = false;
    }
  }
  target_velocity = 24.0;
  setFollowPathVelocity(target_velocity);
  delay(200);
  m_control_system->pause();
  // Wait for the blue robot to leave, robot is too zoomy
  current_time = getTime();
  waitForOpposingRing(2500);
  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // regrab goal to get better clamp
  setClamp(false);
  position = getOdomPosition();
  driveStraight(-12.0, target_velocity, position.theta);
  waitForDriveStraight(-12.0, 1000, 0.5);
  delay(75);
  setClamp(true);
  driveStraight(12.0, target_velocity, position.theta);
  waitForDriveStraight(12.0, 1000, 0.5);
  m_control_system->pause();
  m_delayer->delayUntil(current_time + 3000);
  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);

  // go to ring by wall stake
  std::vector<control::Point> wall_stake_rings_control_points{};
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    wall_stake_rings_control_points = std::vector<control::Point>{
        control::Point{90.0, 97.0}, control::Point{101.5, 112.0},
        control::Point{122.0, 102.5}, control::Point{121.0, 78.0}};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    wall_stake_rings_control_points =
        std::vector<control::Point>{control::Point{144.0 - 88.0, 96.0},
                                    control::Point{144.0 - 101.5, 112.0},
                                    control::Point{144.0 - 122.0, 102.5},
                                    control::Point{144.0 - 123.0, 77.0}};
  std::vector<control::Point> wall_stake_rings_path{
      control::path::BezierCurveInterpolation::calculate(
          wall_stake_rings_control_points)};
  target_point = wall_stake_rings_control_points.back();
  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());

  target_velocity = 36.0;
  setFollowPathVelocity(target_velocity);
  followPath(wall_stake_rings_path, target_velocity);

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    while (target_distance > 28.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
    setIntakeHeight(true);
  }

  while (target_distance > 12.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 12.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 0.5);

  position = getOdomPosition();

  m_control_system->pause();
  setIntakeHeight(false);
  waitForAllianceRing(1000);
  delay(350);

  // Go towards next ring stack
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{112.0, 114.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 112.0, 114.0};

  target_velocity = 60.0;
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

  // point intake towards rings
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
                control::motion::ETurnDirection::COUNTERCLOCKWISE);
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity,
                control::motion::ETurnDirection::CLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 2000,
                     M_PI / 20.0);

  // resume elevator and intake
  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);
  setIntakeHeight(true);

  // finish motion to rings
  target_velocity = 48.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 0.5);
  setIntakeHeight(false);
  m_control_system->pause();

  // intake the ring stack
  waitForOpposingRing(600);
  position = getOdomPosition();
  target_velocity = 36.0;
  driveStraight(15.0, target_velocity, position.theta);
  waitForDriveStraight(15.0, 1000, 0.5);
  m_control_system->pause();
  waitForOpposingRing(500);

  // turn to corner
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{144.0, 144.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 150.0, 144.0};

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 20.0);

  setIntakeHeight(true);

  target_velocity = 20.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);
  position = getOdomPosition();

  setIntakeHeight(false);

  // clear corner
  target_velocity = 20.0;
  while (current_time < start_time + 19500) {
    driveStraight(5, target_velocity, position.theta);
    waitForDriveStraight(5, 600, 0.25);
    m_control_system->pause();
    delay(100);
    driveStraight(-4.0, target_velocity, position.theta);
    waitForDriveStraight(-4.0, 500, 0.25);
    m_control_system->pause();
    delay(100);
    current_time = getTime();
  }

  driveStraight(-12.0, target_velocity, M_PI / 4.0);
  waitForDriveStraight(-12.0, 2000, 0.5);

  // go to the rings by alliance stake

  target_velocity = 72.0;
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{65.0, 130.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 68.5, 127.5};

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 25.0);
  setElevatorVoltage(8.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1700, 0.5);
  waitForAllianceRing(1500);
  armGoLoad();

  // turn to face alliance stake
  turnToAngle(M_PI / 2.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(M_PI / 2.0, 1000, M_PI / 25.0);

  target_velocity = 12.0;
  driveStraight(12.0, target_velocity, M_PI / 2.0);
  waitForDriveStraight(12.0, 500, 0.5);
  m_control_system->pause();

  delay(750);
  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  armGoAllianceStake();
  m_delayer->delay(600);

  target_velocity = 20.0;
  driveStraight(-16.0, target_velocity, M_PI / 2.0);
  delay(1000);
  armGoNeutral();
  waitForDriveStraight(-16.0, 1000, 0.5);

  // touch ladder for AWP
  turnToAngle(3.0 * M_PI / 2.0, target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(3.0 * M_PI / 2.0, 700, M_PI / 25.0);

  driveStraight(24.0, target_velocity, 3.0 * M_PI / 2.0);
  delay(1000);
  armGoAllianceStake();
  m_control_system->pause();

  // display the runtime at the end
  current_time = getTime();
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "Runtime: %7.2f",
                      (current_time - start_time) / 1000.0);
}
}  // namespace auton
}  // namespace driftless