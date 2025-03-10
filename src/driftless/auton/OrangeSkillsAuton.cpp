#include "driftless/auton/OrangeSkillsAuton.hpp"

#include "pros/screen.hpp"

namespace driftless {
namespace auton {
void OrangeSkillsAuton::startColorSort() {
  m_process_system->sendCommand(
      processes::EProcess::AUTO_RING_REJECTION,
      processes::EProcessCommand::AUTO_RING_REJECTION_REJECT_RINGS, &m_robot,
      &m_alliance);
}

void OrangeSkillsAuton::pauseColorSort() {
  m_process_system->pause(processes::EProcess::AUTO_RING_REJECTION);
}

void OrangeSkillsAuton::resumeColorSort() {
  m_process_system->resume(processes::EProcess::AUTO_RING_REJECTION);
}

void OrangeSkillsAuton::calibrateArm() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_CALIBRATE);
}

void OrangeSkillsAuton::armGoNeutral() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
}

void OrangeSkillsAuton::armGoLoad() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
}

void OrangeSkillsAuton::armGoAllianceStake() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemCommand::ARM_GO_ALLIANCE_STAKE);
}

void OrangeSkillsAuton::armGoScore() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_SCORE);
}

void OrangeSkillsAuton::armGoReady() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_READY);
}

void OrangeSkillsAuton::setClamp(bool clamped) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAMP,
                       robot::subsystems::ESubsystemCommand::CLAMP_SET_STATE,
                       clamped);
}

void OrangeSkillsAuton::setElevatorVoltage(double voltage) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_SET_VOLTAGE, voltage);
}

void OrangeSkillsAuton::waitForAllianceRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasAllianceRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void OrangeSkillsAuton::waitForOpposingRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasOpposingRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void OrangeSkillsAuton::setIntakeVoltage(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_VOLTAGE,
                       voltage);
}

void OrangeSkillsAuton::setIntakeHeight(bool high) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_HEIGHT,
                       high);
}

void OrangeSkillsAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemCommand::ODOMETRY_SET_POSITION, x, y, theta);
}

void OrangeSkillsAuton::odomResetX() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ODOMETRY,
                       robot::subsystems::ESubsystemCommand::ODOMETRY_RESET_X);
}

void OrangeSkillsAuton::odomResetY() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ODOMETRY,
                       robot::subsystems::ESubsystemCommand::ODOMETRY_RESET_Y);
}

void OrangeSkillsAuton::followPath(std::vector<control::Point>& path,
                                   double velocity) {
  m_control_system->sendCommand(control::EControl::PATH_FOLLOWER,
                                control::EControlCommand::FOLLOW_PATH, &m_robot,
                                path, velocity);
}

void OrangeSkillsAuton::setFollowPathVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::PATH_FOLLOWER,
      control::EControlCommand::PATH_FOLLOWER_SET_VELOCITY, velocity);
}

void OrangeSkillsAuton::goToPoint(double x, double y, double velocity) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::GO_TO_POINT, &m_robot,
                                velocity, x, y);
}

void OrangeSkillsAuton::setGoToPointVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::EControlCommand::GO_TO_POINT_SET_VELOCITY, velocity);
}

void OrangeSkillsAuton::waitForGoToPoint(double target_x, double target_y,
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

void OrangeSkillsAuton::turnToPoint(double x, double y, double velocity,
                                    control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_POINT,
                                &m_robot, velocity, x, y, direction);
}

void OrangeSkillsAuton::waitForTurnToPoint(double x, double y, uint32_t timeout,
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

void OrangeSkillsAuton::turnToAngle(double theta, double velocity,
                                    control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_ANGLE,
                                &m_robot, velocity, theta, direction);
}

void OrangeSkillsAuton::waitForTurnToAngle(double theta, uint32_t timeout,
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

void OrangeSkillsAuton::driveStraight(double distance, double velocity,
                                      double theta) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::DRIVE_STRAIGHT,
                                &m_robot, velocity, distance, theta);
}

void OrangeSkillsAuton::waitForDriveStraight(double target_distance,
                                             uint32_t timeout,
                                             double tolerance) {
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

void OrangeSkillsAuton::delay(uint32_t delay_time) {
  uint32_t current_time{getTime()};
  uint32_t end_time{current_time + delay_time};

  while (current_time < end_time) {
    current_time = getTime();
    m_delayer->delay(LOOP_DELAY);
  }
}

uint32_t OrangeSkillsAuton::getTime() {
  uint32_t current_time{};
  if (m_clock) {
    current_time = m_clock->getTime();
  }
  return current_time;
}

bool OrangeSkillsAuton::isArmReady() {
  return *(static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::ARM,
                        robot::subsystems::ESubsystemState::ARM_IS_READY)));
}

robot::subsystems::odometry::Position OrangeSkillsAuton::getOdomPosition() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  return position;
}

double OrangeSkillsAuton::getOdomVelocity() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  double velocity{
      std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2))};
  return velocity;
}

bool OrangeSkillsAuton::followPathTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::PATH_FOLLOWER,
      control::EControlState::PATH_FOLLOWER_TARGET_REACHED))};
  return target_reached;
}

bool OrangeSkillsAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POINT_TARGET_REACHED))};
  return target_reached;
}

bool OrangeSkillsAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION, control::EControlState::TURN_TARGET_REACHED))};
  return target_reached;
}

bool OrangeSkillsAuton::driveStraightTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::DRIVE_STRAIGHT_TARGET_REACHED))};
  return target_reached;
}

bool OrangeSkillsAuton::hasAllianceRing() {
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

bool OrangeSkillsAuton::hasOpposingRing() {
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

std::string OrangeSkillsAuton::getName() { return AUTON_NAME; }

void OrangeSkillsAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system,
    std::shared_ptr<driftless::processes::ProcessSystem>& process_system) {
  m_robot = robot;
  m_control_system = control_system;
  m_process_system = process_system;
}

void OrangeSkillsAuton::run(
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
  setOdomPosition(72.0, 132.0, 3 * M_PI / 2.0);
  robot::subsystems::odometry::Position position{getOdomPosition()};
  double velocity{getOdomVelocity()};

  uint32_t current_time{start_time};
  control::Point target_point{};
  double target_distance{};
  double target_velocity{};
  double target_angular_velocity{M_PI * 1.5};

  // start the color sorter
  startColorSort();

  // calibrate the arm
  calibrateArm();

  // grab ring in front of robot for alliance stake
  setIntakeVoltage(12.0);
  setElevatorVoltage(12.0);

  target_velocity = 32.0;
  driveStraight(6.0, target_velocity, 3 * M_PI / 2.0);
  waitForDriveStraight(6.0, 1000, 0.5);

  // turn to alliance stake
  turnToAngle(M_PI / 2.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(M_PI / 2.0, 1500, M_PI / 25.0);

  // drive to alliance stake
  driveStraight(12.0, target_velocity, M_PI / 2.0);
  waitForDriveStraight(12.0, 500, 0.5);
  m_control_system->pause();

  // score ring?
  waitForAllianceRing(300);
  delay(400);
  armGoAllianceStake();

  setIntakeVoltage(0.0);
  setElevatorVoltage(0.0);
  delay(750);

  // drive away to get ring on stake
  target_velocity = 36.0;

  goToPoint(72.0, 120.0, target_velocity);
  delay(300);
  armGoLoad();
  waitForGoToPoint(72.0, 120.0, 1500, 0.5);
  m_control_system->pause();

  armGoNeutral();

  // go to first ring for stack 1
  target_point = control::Point{52.0, 101.0};
  target_velocity = 34.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::COUNTERCLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 32.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  setIntakeVoltage(12.0);
  setElevatorVoltage(12.0);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 0.5);

  waitForAllianceRing(500);
  setElevatorVoltage(-4.0);
  setIntakeVoltage(0.0);

  delay(150);

  setElevatorVoltage(0.0);

  // turn to mogo 1
  target_point = control::Point{27.0, 98.0};
  target_velocity = 25.0;

  turnToAngle(0, target_angular_velocity,
              control::motion::ETurnDirection::COUNTERCLOCKWISE);
  waitForTurnToAngle(0, 1500, M_PI / 32.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 0.25);
  m_control_system->pause();

  // grab mogo 1
  setClamp(true);
  setElevatorVoltage(12.0);
  delay(500);

  // go to ring under ladder for stack 1
  target_point = control::Point{66.0, 83.0};
  target_velocity = 30.0;

  setIntakeVoltage(12.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 0.25);
  m_control_system->pause();

  waitForAllianceRing(750);

  setElevatorVoltage(-6.0);
  delay(150);
  setElevatorVoltage(0.0);

  // back up to turn around for ring 3 of stack 1
  target_point = control::Point{48.0, 96.0};
  target_velocity = 32.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 2.5);
  m_control_system->pause();

  setElevatorVoltage(9.0);
  delay(50);
  setElevatorVoltage(12.0);
  delay(500);

  // turn to third ring for stack 1
  target_point = control::Point{28.0, 76.0};
  target_velocity = 36.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::CLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 8.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1250, 1.0);
  m_control_system->pause();

  waitForAllianceRing(250);
  delay(600);

  // turn to ring 4 for stack 1

  target_point = control::Point{24.0, 120.0};
  target_velocity = 38.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::CLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 8.0);

  delay(50);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.0);
  m_control_system->pause();

  waitForAllianceRing(500);
  delay(250);

  // turn to ring 5 for stack 1
  target_point = control::Point{6.0, 140.0};
  target_velocity = 24.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1250,
                     M_PI / 20.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.0);
  m_control_system->pause();

  waitForAllianceRing(500);
  delay(500);

  setIntakeVoltage(0.0);

  // back up and turn around for stack 1

  target_point = control::Point{24.0, 116.0};
  target_velocity = 42.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1250, 2.5);

  turnToAngle(-M_PI / 4.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(-M_PI / 4.0, 1500, M_PI / 32.0);

  // put stack 1 in corner

  target_point = control::Point{10.0, 138.0};
  target_velocity = 25.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1250, 1.0);
  m_control_system->pause();

  setClamp(false);

  // drive out of corner

  target_velocity = 32.0;

  driveStraight(12.0, target_velocity, -M_PI / 4.0);
  waitForDriveStraight(12.0, 1000, 1.0);

  // go to wall stake rings

  target_point = control::Point{16.0, 76.0};
  target_velocity = 40.0;

  setIntakeVoltage(12.0);
  setElevatorVoltage(12.0);
  armGoLoad();

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 32.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 0.1);

  // turn to wall stake

  turnToAngle(M_PI, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(M_PI, 1500, M_PI / 100.0);

  waitForAllianceRing(500);
  delay(250);

  setElevatorVoltage(0.0);
  armGoReady();

  while (!isArmReady()) {
    delay(LOOP_DELAY);
  }
  driveStraight(8.0, target_velocity, M_PI);
  waitForDriveStraight(8.0, 500, 1.0);

  armGoScore();
  delay(200);

  // back up to turn for first ring of stack 2

  target_point = control::Point{18.0, 72.0};
  target_velocity = 24.0;

  setElevatorVoltage(12.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 250, 1.0);

  // turn and go to first ring stack for stack 2

  target_point = control::Point{24.0, 48.0};
  target_velocity = 24.0;

  armGoNeutral();

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::COUNTERCLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 20.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 1.0);

  waitForAllianceRing(1000);
  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // grab mogo 2

  target_point = control::Point{56.0, 50.0};
  target_velocity = 24.0;

  turnToAngle(M_PI, target_angular_velocity,
              control::motion::ETurnDirection::CLOCKWISE);
  waitForTurnToAngle(M_PI, 1500, M_PI / 32.0);

  setElevatorVoltage(0.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 0.1);
  m_control_system->pause();

  setClamp(true);
  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);

  delay(250);

  // grab ring under ladder for stack 2

  target_point = control::Point{71.0, 71.0};
  target_velocity = 42.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::CLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 32.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1250, 1.0);
  m_control_system->pause();

  delay(500);

  // spin and drop stack 2

  target_point = control::Point{0.0, 0.0};

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 32.0);

  setClamp(false);

  // grab first ring for stack 3

  target_point = control::Point{44.0, 22.0};
  target_velocity = 40.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 1.0);

  waitForAllianceRing(750);
  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // go to mogo 3

  target_point = control::Point{72.0, 31.0};
  target_velocity = 24.0;

  turnToAngle(M_PI, target_angular_velocity,
              control::motion::ETurnDirection::CLOCKWISE);
  waitForTurnToAngle(M_PI, 1500, M_PI / 32.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 0.1);
  m_control_system->pause();

  setClamp(true);
  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);

  delay(300);

  // pick up ring 2 for stack 3

  target_point = control::Point{24.0, 28.0};
  target_velocity = 36.0;

  setIntakeHeight(true);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2500, 1.0);

  setIntakeHeight(false);

  position = getOdomPosition();
  target_velocity = 24.0;

  driveStraight(12.0, target_velocity, position.theta);

  waitForAllianceRing(1000);
  delay(500);

  // back up to go towards ring 3 for stack 3

  target_point = control::Point{28.0, 24.0};
  target_velocity = 24.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 1.0);

  // pick up ring 3 for stack 3

  target_point = control::Point{-10.0, 0.0};
  target_velocity = 28.0;

  setIntakeHeight(true);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.0);

  setIntakeHeight(false);

  delay(500);

  // back up and secure ring 3 for stack 3

  target_distance = 10.0;
  position = getOdomPosition();

  setIntakeVoltage(-12.0);
  driveStraight(-target_distance, target_velocity, position.theta);
  waitForDriveStraight(-target_distance, 1000, 1.0);

  delay(100);

  setIntakeVoltage(12.0);
  driveStraight(target_distance + 2.0, target_velocity, position.theta);
  waitForDriveStraight(target_distance + 2.0, 1000, 1.0);

  // back up to turn around for stack 3

  target_point = control::Point{24.0, 24.0};
  target_velocity = 24.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 1.0);

  turnToAngle(M_PI / 4.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(M_PI / 4.0, 1500, M_PI / 32.0);

  // drop stack 3 in corner

  target_point = control::Point{0.0, 0.0};
  target_velocity = 30.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 1.0);

  setClamp(false);
  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);

  // drive away so goal can be flat

  target_distance = 24.0;

  driveStraight(target_distance, target_velocity, M_PI / 4.0);
  waitForDriveStraight(target_distance, 1000, 1.0);
  m_control_system->pause();

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