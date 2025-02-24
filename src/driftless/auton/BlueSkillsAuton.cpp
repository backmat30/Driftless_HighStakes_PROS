#include "driftless/auton/BlueSkillsAuton.hpp"

#include "pros/screen.hpp"

namespace driftless {
namespace auton {
void BlueSkillsAuton::startColorSort() {
  m_process_system->sendCommand(
      processes::EProcess::AUTO_RING_REJECTION,
      processes::EProcessCommand::AUTO_RING_REJECTION_REJECT_RINGS, m_robot,
      m_alliance);
}

void BlueSkillsAuton::pauseColorSort() {
  m_process_system->pause(processes::EProcess::AUTO_RING_REJECTION);
}

void BlueSkillsAuton::resumeColorSort() {
  m_process_system->resume(processes::EProcess::AUTO_RING_REJECTION);
}

void BlueSkillsAuton::calibrateArm() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_CALIBRATE);
}

void BlueSkillsAuton::armGoNeutral() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
}

void BlueSkillsAuton::armGoLoad() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
}

void BlueSkillsAuton::armGoAllianceStake() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemCommand::ARM_GO_ALLIANCE_STAKE);
}

void BlueSkillsAuton::armGoScore() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_SCORE);
}

void BlueSkillsAuton::armGoReady() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_READY);
}

void BlueSkillsAuton::setClamp(bool clamped) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAMP,
                       robot::subsystems::ESubsystemCommand::CLAMP_SET_STATE,
                       clamped);
}

void BlueSkillsAuton::setElevatorVoltage(double voltage) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_SET_VOLTAGE, voltage);
}

void BlueSkillsAuton::waitForAllianceRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasAllianceRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void BlueSkillsAuton::waitForOpposingRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasOpposingRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void BlueSkillsAuton::setIntakeVoltage(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_VOLTAGE,
                       voltage);
}

void BlueSkillsAuton::setIntakeHeight(bool high) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_HEIGHT,
                       high);
}

void BlueSkillsAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemCommand::ODOMETRY_SET_POSITION, x, y, theta);
}

void BlueSkillsAuton::odomResetX() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ODOMETRY,
                       robot::subsystems::ESubsystemCommand::ODOMETRY_RESET_X);
}

void BlueSkillsAuton::odomResetY() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ODOMETRY,
                       robot::subsystems::ESubsystemCommand::ODOMETRY_RESET_Y);
}

void BlueSkillsAuton::followPath(std::vector<control::Point>& path,
                                 double velocity) {
  m_control_system->sendCommand(control::EControl::PATH_FOLLOWER,
                                control::EControlCommand::FOLLOW_PATH, &m_robot,
                                path, velocity);
}

void BlueSkillsAuton::setFollowPathVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::PATH_FOLLOWER,
      control::EControlCommand::PATH_FOLLOWER_SET_VELOCITY, velocity);
}

void BlueSkillsAuton::goToPoint(double x, double y, double velocity) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::GO_TO_POINT, &m_robot,
                                velocity, x, y);
}

void BlueSkillsAuton::setGoToPointVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::EControlCommand::GO_TO_POINT_SET_VELOCITY, velocity);
}

void BlueSkillsAuton::waitForGoToPoint(double target_x, double target_y,
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

void BlueSkillsAuton::turnToPoint(double x, double y, double velocity,
                                  control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_POINT,
                                &m_robot, velocity, x, y, direction);
}

void BlueSkillsAuton::waitForTurnToPoint(double x, double y, uint32_t timeout,
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

void BlueSkillsAuton::turnToAngle(double theta, double velocity,
                                  control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_ANGLE,
                                &m_robot, velocity, theta, direction);
}

void BlueSkillsAuton::waitForTurnToAngle(double theta, uint32_t timeout,
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

void BlueSkillsAuton::driveStraight(double distance, double velocity,
                                    double theta) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::DRIVE_STRAIGHT,
                                &m_robot, velocity, distance, theta);
}

void BlueSkillsAuton::waitForDriveStraight(double target_distance,
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

void BlueSkillsAuton::delay(uint32_t delay_time) {
  uint32_t current_time{getTime()};
  uint32_t end_time{current_time + delay_time};

  while (current_time < end_time) {
    current_time = getTime();
    m_delayer->delay(LOOP_DELAY);
  }
}

uint32_t BlueSkillsAuton::getTime() {
  uint32_t current_time{};
  if (m_clock) {
    current_time = m_clock->getTime();
  }
  return current_time;
}

bool BlueSkillsAuton::isArmReady() {
  return *(static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_READY)));
}

robot::subsystems::odometry::Position BlueSkillsAuton::getOdomPosition() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  return position;
}

double BlueSkillsAuton::getOdomVelocity() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  double velocity{
      std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2))};
  return velocity;
}

bool BlueSkillsAuton::followPathTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::PATH_FOLLOWER,
      control::EControlState::PATH_FOLLOWER_TARGET_REACHED))};
  return target_reached;
}

bool BlueSkillsAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POINT_TARGET_REACHED))};
  return target_reached;
}

bool BlueSkillsAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION, control::EControlState::TURN_TARGET_REACHED))};
  return target_reached;
}

bool BlueSkillsAuton::driveStraightTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::DRIVE_STRAIGHT_TARGET_REACHED))};
  return target_reached;
}

bool BlueSkillsAuton::hasAllianceRing() {
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

bool BlueSkillsAuton::hasOpposingRing() {
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

std::string BlueSkillsAuton::getName() { return AUTON_NAME; }

void BlueSkillsAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system,
    std::shared_ptr<driftless::processes::ProcessSystem>& process_system) {
  m_robot = robot;
  m_control_system = control_system;
  m_process_system = process_system;
}

void BlueSkillsAuton::run(
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

  uint32_t start_time{getTime()};
  setOdomPosition(103.0, 129, M_PI / 2.0);
  robot::subsystems::odometry::Position position{getOdomPosition()};
  double velocity{getOdomVelocity()};

  uint32_t current_time{start_time};
  control::Point target_point{};
  double target_distance{};
  double target_velocity{};
  double target_angular_velocity{M_PI * 1.65};

  // start the color sorter
  startColorSort();

  // calibrate the arm
  calibrateArm();

  // grab mogo 1

  target_point = control::Point{119.0, 99.0};
  target_velocity = 32.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 0.25);

  setClamp(true);

  armGoNeutral();

  // go to first ring for stack 1

  setIntakeVoltage(12.0);
  setElevatorVoltage(12.0);

  target_point = control::Point{96.0, 96.0};
  target_velocity = 48.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);

  waitForAllianceRing(750);
  delay(250);

  // go to ring 2 for stack 1

  target_point = control::Point{81.0, 81.0};
  target_velocity = 36.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1250,
                     M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);
  m_control_system->pause();

  waitForAllianceRing(1000);
  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // back up before putting ring on

  target_point = control::Point{100.0, 100.0};
  target_velocity = 48.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);
  m_control_system->pause();

  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);

  delay(500);

  // go to ring 3 for stack 1

  target_point = control::Point{116.0, 78.0};
  target_velocity = 40.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 0.25);
  m_control_system->pause();

  delay(500);

  // go to ring 4 for stack 1

  target_point = control::Point{120.0, 120.0};
  target_velocity = 48.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);
  m_control_system->pause();

  delay(200);

  // go to ring 5 for stack 1

  target_point = control::Point{144.0, 144.0};
  target_velocity = 30.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.0);

  // back up and spin

  target_distance = 24.0;
  target_velocity = 28.0;

  driveStraight(-target_distance, target_velocity, M_PI / 4.0);
  waitForDriveStraight(-target_distance, 1500, 1.0);

  turnToAngle(-3.0 * M_PI / 4.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(-3.0 * M_PI / 4.0, 1500, M_PI / 10.0);

  // put stack 1 in corner

  driveStraight(-target_distance, target_velocity, -3.0 * M_PI / 4.0);
  waitForDriveStraight(-target_distance, 1300, 1.0);
  m_control_system->pause();

  setClamp(false);
  delay(150);

  driveStraight(target_distance, target_velocity, -2.0 * M_PI / 3.0);
  waitForDriveStraight(target_distance, 1500, 1.0);
  m_control_system->pause();

  // go to wall stake rings

  target_point = control::Point{131.0, 77.0};
  target_velocity = 48.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1250,
                     M_PI / 10.0);

  armGoLoad();

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 0.25);

  waitForAllianceRing(1000);
  delay(500);

  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // turn and score wall stake
  target_distance = 12.0;
  target_velocity = 20.0;

  turnToAngle(0.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(0.0, 750, M_PI / 10.0);

  armGoReady();
  delay(400);

  driveStraight(target_distance, target_velocity, 0.0);
  waitForDriveStraight(target_distance, 750, 1.0);

  armGoScore();
  delay(400);

  driveStraight(-target_distance, target_velocity, 0.0);
  waitForDriveStraight(-target_distance, 2000, 1.0);

  armGoNeutral();

  // go to ring 1 for stack 2

  target_point = control::Point{122.0, 57.0};
  target_velocity = 30.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 2000,
                     M_PI / 10.0);

  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);

  waitForAllianceRing(2000);

  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // go to mogo 2

  target_point = control::Point{98.0, 52.0};
  target_velocity = 24.0;

  turnToAngle(0.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(0.0, 1500, M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 0.25);

  setClamp(true);

  delay(150);

  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);

  // get ring 2 for stack 2

  target_point = control::Point{81.0, 66.0};
  target_velocity = 40.0;

  turnToPoint(target_point.getX() + 9.0, target_point.getY(),
              target_angular_velocity, control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500, M_PI / 10);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.0);
  m_control_system->pause();

  waitForAllianceRing(1000);

  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // back up out from the ladder

  target_point = control::Point{98.0, 52.0};
  target_velocity = 40.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);

  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);

  delay(250);

  // go to ring 3 for stack 2

  target_point = control::Point{100.0, 30.0};
  target_velocity = 28.0;

  setIntakeHeight(true);

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 2000,
                     M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.0);
  m_control_system->pause();

  setIntakeHeight(false);

  waitForAllianceRing(2000);
  delay(500);

  // go to ring 4 for stack 2

  target_point = control::Point{126.0, 32.0};
  target_velocity = 42.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.0);
  m_control_system->pause();

  waitForAllianceRing(1000);

  // secure ring 4

  target_distance = 12.0;
  target_velocity = 30.0;

  driveStraight(target_distance, target_velocity, 0.0);
  waitForDriveStraight(target_distance, 1500, 1.0);

  waitForOpposingRing(1000);

  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  driveStraight(-target_distance, target_velocity, 0.0);
  waitForDriveStraight(-target_distance, 1500, 1.0);

  // spit out blue ring fron ring 4

  target_distance = 24.0;

  turnToAngle(M_PI / 2.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(M_PI / 2.0, 1250, M_PI / 10.0);

  driveStraight(target_distance, target_velocity, M_PI / 2.0);
  waitForDriveStraight(target_distance, 2000, 1.0);

  setElevatorVoltage(-12.0);
  setIntakeVoltage(-12.0);

  delay(50);

  driveStraight(-target_distance / 4.0, target_velocity, M_PI / 2.0);
  waitForDriveStraight(-target_distance / 4.0, 1500, M_PI / 2.0);

  // go to ring 5 for stack 2

  target_point = control::Point{152.0, 4.0};
  target_velocity = 30.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 2000,
                     M_PI / 10.0);

  setIntakeHeight(true);
  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 1.0);
  m_control_system->pause();

  setIntakeHeight(false);

  delay(250);

  // secure ring 5 for stack 2

  target_distance = 12.0;
  target_velocity = 24.0;
  position = getOdomPosition();

  driveStraight(-target_distance, target_velocity, position.theta);
  waitForDriveStraight(-target_distance, 1500, 1.0);

  setIntakeVoltage(-12.0);

  delay(150);

  setIntakeVoltage(12.0);

  driveStraight(target_distance, target_velocity, position.theta);
  waitForDriveStraight(target_distance, 2000, 1.0);

  waitForAllianceRing(1000);
  delay(400);

  driveStraight(-target_distance, target_velocity, position.theta);
  waitForDriveStraight(-target_distance, 1000, 1.0);

  // back up and turn to put mogo 2 in corner

  target_velocity = 32.0;

  driveStraight(-target_distance, target_velocity, -M_PI / 4.0);
  waitForDriveStraight(-target_distance, 2000, 1.0);

  turnToAngle(3 * M_PI / 4.0, target_angular_velocity,
              control::motion::ETurnDirection::COUNTERCLOCKWISE);
  waitForTurnToAngle(3 * M_PI / 4.0, 2000, M_PI / 10.0);

  driveStraight(-target_distance, target_velocity, 2.0 * M_PI / 3.0);
  waitForDriveStraight(-target_distance, 1250, 1.0);

  setClamp(false);

  delay(100);

  driveStraight(target_distance - 4.0, target_velocity, 3 * M_PI / 4.0);
  waitForDriveStraight(target_distance, 2000, 1.0);
  m_control_system->pause();

  // go to alliance stake ring

  target_point = control::Point{82.0, 15.0};
  target_velocity = 40.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 10.0);

  armGoLoad();

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 0.25);
  m_control_system->pause();

  waitForAllianceRing(750);

  // score on alliance stake

  target_distance = 12.0;
  target_velocity = 32.0;

  turnToAngle(-M_PI / 2.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(-M_PI / 2.0, 1500, M_PI / 10.0);

  driveStraight(target_distance, target_velocity, -M_PI / 2.0);
  waitForDriveStraight(target_distance, 1500, 1.0);
  m_control_system->pause();

  armGoAllianceStake();

  delay(750);

  // drive away to get ring on stake

  target_velocity = 42.0;

  driveStraight(-target_distance, target_velocity, -M_PI / 2.0);

  // moves arm to look cool and not abuse the stupid ring bs that john wants
  /*
  delay(350);

  armGoNeutral();
  */

  waitForDriveStraight(-target_distance, 1500, 1.0);
  m_control_system->pause();

  setIntakeVoltage(0.0);
  setElevatorVoltage(0.0);

  /*
   *  KEEP AT END
   */

  // display the runtime at the end
  current_time = getTime();
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 5, "Runtime: %7.2f",
                      (current_time - start_time) / 1000.0);
}
}  // namespace auton
}  // namespace driftless