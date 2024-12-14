#include "driftless/auton/OrangeRushAuton.hpp"

#include "pros/screen.hpp"

namespace driftless {
namespace auton {
void OrangeRushAuton::calibrateArm() {
  m_robot->sendCommand(ARM_SUBSYSTEM_NAME, ARM_CALIBRATE_COMMAND);
}

void OrangeRushAuton::armGoNeutral() {
  m_robot->sendCommand(ARM_SUBSYSTEM_NAME, ARM_GO_NEUTRAL_COMMAND);
}

void OrangeRushAuton::armGoLoad() {
  m_robot->sendCommand(ARM_SUBSYSTEM_NAME, ARM_GO_LOAD_COMMAND);
}

void OrangeRushAuton::armGoAllianceStake() {
  m_robot->sendCommand(ARM_SUBSYSTEM_NAME, ARM_GO_ALLIANCE_COMMAND);
}

void OrangeRushAuton::setClamp(bool clamped) {
  m_robot->sendCommand(CLAMP_SUBSYSTEM_NAME, CLAMP_SET_STATE_COMMAND, clamped);
}

void OrangeRushAuton::setElevatorVoltage(double voltage) {
  m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME, ELEVATOR_SET_VOLTAGE, voltage);
}

void OrangeRushAuton::updateRingSort() {
  void* position_state{
      m_robot->getState(ELEVATOR_SUBSYSTEM_NAME, ELEVATOR_POSITION_STATE)};
  double position{*static_cast<double*>(position_state)};

  void* distance_to_end_state{m_robot->getState(
      RING_SORT_SUBSYSTEM_NAME, RING_SORT_GET_DISTANCE_TO_END_STATE)};
  double distance_to_end{*static_cast<double*>(distance_to_end_state)};

  if (hasOpposingRing()) {
    ring_sort_latest_ring_pos = position;
  }
  if (hasAllianceRing()) {
    ring_sort_latest_ring_pos = -__DBL_MAX__;
  }

  if (position <= ring_sort_latest_ring_pos + distance_to_end &&
      position >= ring_sort_latest_ring_pos - 0.1) {
    m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME,
                         ELEVATOR_DEPLOY_REJECTOR_COMMAND);
  } else {
    m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME,
                         ELEVATOR_RETRACT_REJECTOR_COMMAND);
    ring_sort_latest_ring_pos = -__DBL_MAX__;
  }
}

void OrangeRushAuton::waitForAllianceRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasAllianceRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
    updateRingSort();
  }
}

void OrangeRushAuton::waitForOpposingRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasOpposingRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
    updateRingSort();
  }
}

void OrangeRushAuton::spinIntake(double voltage) {
  m_robot->sendCommand(INTAKE_SUBSYSTEM_NAME, INTAKE_SPIN_COMMAND, voltage);
}

void OrangeRushAuton::setIntakeHeight(bool high) {
  m_robot->sendCommand(INTAKE_SUBSYSTEM_NAME, INTAKE_SET_HEIGHT_COMMAND, high);
}

void OrangeRushAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(ODOMETRY_SUBSYSTEM_NAME, ODOMETRY_SET_POSITION_COMMAND,
                       x, y, theta);
}

void OrangeRushAuton::followPath(std::vector<control::Point>& path,
                                 double velocity) {
  m_control_system->sendCommand(PATH_FOLLOWER_CONTROL_NAME, FOLLOW_PATH_COMMAND,
                                &m_robot, path, velocity);
}

void OrangeRushAuton::setFollowPathVelocity(double velocity) {
  m_control_system->sendCommand(PATH_FOLLOWER_CONTROL_NAME,
                                SET_PATH_FOLLOWER_VELOCTY_COMMAND, velocity);
}

void OrangeRushAuton::goToPoint(double x, double y, double velocity) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME, GO_TO_POINT_COMMAND,
                                &m_robot, velocity, x, y);
}

void OrangeRushAuton::setGoToPointVelocity(double velocity) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME,
                                SET_GO_TO_POINT_VELOCITY_COMMAND, velocity);
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
    updateRingSort();
    m_delayer->delay(LOOP_DELAY);
  }
}

void OrangeRushAuton::turnToPoint(double x, double y, double velocity,
                                  control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME, TURN_TO_POINT_COMMAND,
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
    updateRingSort();
    m_delayer->delay(LOOP_DELAY);
  }
}

void OrangeRushAuton::turnToAngle(double theta, double velocity,
                                  control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME, TURN_TO_ANGLE_COMMAND,
                                &m_robot, velocity, theta, direction);
}

void OrangeRushAuton::waitForTurnToAngle(double theta, uint32_t timeout,
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

void OrangeRushAuton::driveStraight(double distance, double velocity,
                                    double theta) {
  m_control_system->sendCommand(MOTION_CONTROL_NAME, DRIVE_STRAIGHT_COMMAND,
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
    updateRingSort();
    m_delayer->delay(LOOP_DELAY);
  }
}

void OrangeRushAuton::delay(uint32_t delay_time) {
  uint32_t current_time{getTime()};
  uint32_t end_time{current_time + delay_time};

  while (current_time < end_time) {
    current_time = getTime();
    updateRingSort();
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
          ODOMETRY_SUBSYSTEM_NAME, ODOMETRY_GET_POSITION_STATE))};

  return position;
}

double OrangeRushAuton::getOdomVelocity() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          ODOMETRY_SUBSYSTEM_NAME, ODOMETRY_GET_POSITION_STATE))};

  double velocity{
      std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2))};
  return velocity;
}

bool OrangeRushAuton::followPathTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      PATH_FOLLOWER_CONTROL_NAME, PATH_FOLLOWER_TARGET_REACHED_STATE))};
  return target_reached;
}

bool OrangeRushAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      MOTION_CONTROL_NAME, GO_TO_POINT_TARGET_REACHED_STATE))};
  return target_reached;
}

bool OrangeRushAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      MOTION_CONTROL_NAME, TURN_TARGET_REACHED_STATE))};
  return target_reached;
}

bool OrangeRushAuton::driveStraightTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      MOTION_CONTROL_NAME, DRIVE_STRAIGHT_TARGET_REACHED_STATE))};
  return target_reached;
}

bool OrangeRushAuton::hasAllianceRing() {
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

bool OrangeRushAuton::hasOpposingRing() {
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
  if (alliance->getName() == "RED")
    setOdomPosition(35.0, 114.5, M_PI / 2.0);
  else if (alliance->getName() == "BLUE")
    setOdomPosition(144.0 - 35.0, 114.5, M_PI / 2.0);
  robot::subsystems::odometry::Position position{getOdomPosition()};
  double velocity{getOdomVelocity()};

  uint32_t current_time{start_time};
  control::Point target_point{};
  double target_distance{};
  double target_velocity{};

  // Start the rush path
  std::vector<control::Point> rush_control_points{};
  if (alliance->getName() == "RED")
    rush_control_points = std::vector<control::Point>{
        control::Point{35.0, 114.5}, control::Point{32.0, 97.0},
        control::Point{31.5, 96.0}, control::Point{24.25, 82.0}};
  else if (alliance->getName() == "BLUE")
    rush_control_points = std::vector<control::Point>{
        control::Point{144.0 - 35.0, 114.5}, control::Point{144.0 - 32.0, 97.0},
        control::Point{144.0 - 31.5, 96.0},
        control::Point{144.0 - 23.75, 81.75}};

  std::vector<control::Point> rush_path{
      control::path::BezierCurveInterpolation::calculate(rush_control_points)};
  target_point = rush_control_points.back();

  target_velocity = 12.0;

  followPath(rush_path, target_velocity);
  // Set up subsystems while moving to the path
  calibrateArm();

  delay(75);
  target_velocity = 24.0;
  setFollowPathVelocity(target_velocity);
  delay(75);
  target_velocity = 40.0;
  setFollowPathVelocity(target_velocity);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 28.5) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 48;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 14.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  setGoToPointVelocity(12.0);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);

  setClamp(true);
  m_control_system->pause();
  m_delayer->delay(100);
  armGoNeutral();

  // Set up the path under the ladder
  std::vector<control::Point> under_ladder_control_points{};
  if (alliance->getName() == "RED")
    under_ladder_control_points = std::vector<control::Point>{
        control::Point{24.0, 82.25},  control::Point{46.0, 97.0},
        control::Point{72.0, 70.0},   control::Point{88.0, 96.0},
        control::Point{101.5, 112.0}, control::Point{122.0, 102.5},
        control::Point{123.0, 80.25}};
  else if (alliance->getName() == "BLUE")
    under_ladder_control_points =
        std::vector<control::Point>{control::Point{144.0 - 24.0, 82.25},
                                    control::Point{144.0 - 46.0, 97.0},
                                    control::Point{144.0 - 72.0, 70.0},
                                    control::Point{144.0 - 88.0, 96.0},
                                    control::Point{144.0 - 101.5, 112.0},
                                    control::Point{144.0 - 122.0, 102.5},
                                    control::Point{144.0 - 123.0, 77.0}};

  std::vector<control::Point> under_ladder_path{
      control::path::BezierCurveInterpolation::calculate(
          under_ladder_control_points)};

  // follow the path under the ladder
  target_point = under_ladder_control_points.at(3);
  target_velocity = 12.0;
  position = getOdomPosition();

  followPath(under_ladder_path, target_velocity);

  spinIntake(12.0);
  setElevatorVoltage(12.0);
  delay(100);
  target_velocity = 30.0;
  setFollowPathVelocity(target_velocity);

  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 24.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 24.0;
  setFollowPathVelocity(target_velocity);

  bool elevator_running{true};
  while (target_distance > 16.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
    updateRingSort();
    if (hasOpposingRing() && elevator_running) {
      setElevatorVoltage(0.0);
      spinIntake(0.0);
      elevator_running = false;
    }
  }
  target_velocity = 10.0;
  setFollowPathVelocity(target_velocity);
  delay(200);
  m_control_system->pause();
  // Wait for the blue robot to leave, robot is too zoomy
  current_time = getTime();
  waitForOpposingRing(2500);
  setElevatorVoltage(0.0);
  spinIntake(0.0);
  m_delayer->delayUntil(current_time + 3000);
  setElevatorVoltage(12.0);
  spinIntake(12.0);

  target_point = under_ladder_path.back();
  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());

  target_velocity = 12.0;
  setFollowPathVelocity(target_velocity);
  m_control_system->resume();
  delay(100);
  target_velocity = 20.0;
  setFollowPathVelocity(target_velocity);

  if (alliance->getName() == "RED") {
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
  driveStraight(-7.0, target_velocity, position.theta);
  waitForDriveStraight(-7.0, 500, 0.5);
  waitForAllianceRing(500);
  if (alliance->getName() == "RED" && !hasAllianceRing()) {
    driveStraight(7.0, target_velocity, position.theta);
    waitForDriveStraight(7.0, 500, 0.5);
  }
  m_control_system->pause();

  spinIntake(0.0);
  setElevatorVoltage(0.0);

  // Go towards next ring stack
  if (alliance->getName() == "RED")
    target_point = control::Point{112.0, 114.0};
  else if (alliance->getName() == "BLUE")
    target_point = control::Point{144.0 - 112.0, 114.0};

  target_velocity = 24.0;
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
  target_velocity = 12.0;
  if (alliance->getName() == "RED")
    turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
                control::motion::ETurnDirection::COUNTERCLOCKWISE);
  else if (alliance->getName() == "BLUE")
    turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
                control::motion::ETurnDirection::CLOCKWISE);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 2000,
                     M_PI / 20.0);

  // resume elevator and intake
  setElevatorVoltage(12.0);
  spinIntake(12.0);
  setIntakeHeight(true);

  // finish motion to rings
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 0.5);
  setIntakeHeight(false);
  m_control_system->pause();

  // intake the ring stack
  waitForOpposingRing(600);
  position = getOdomPosition();
  target_velocity = 24.0;
  driveStraight(15.0, target_velocity, position.theta);
  waitForDriveStraight(15.0, 1000, 0.5);
  m_control_system->pause();
  waitForOpposingRing(500);

  // turn to corner
  if (alliance->getName() == "RED")
    target_point = control::Point{144.0, 144.0};
  else if (alliance->getName() == "BLUE")
    target_point = control::Point{144.0 - 150.0, 144.0};

  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 3000,
                     M_PI / 25.0);

  // grab the alliance ring
  waitForAllianceRing(1000);

  setIntakeHeight(true);

  target_velocity = 16.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);
  position = getOdomPosition();

  setIntakeHeight(false);

  // clear corner
  target_velocity = 12.0;
  while (current_time < start_time + 19500) {
    driveStraight(4.5, target_velocity, position.theta);
    waitForDriveStraight(4.5, 600, 0.25);
    m_control_system->pause();
    delay(100);
    driveStraight(-4.0, target_velocity, position.theta);
    waitForDriveStraight(-4.0, 500, 0.25);
    m_control_system->pause();
    delay(100);
    current_time = getTime();
  }

  // go to the rings by alliance stake

  target_velocity = 12.0;
  if (alliance->getName() == "RED")
    target_point = control::Point{66.5, 128.0};
  else if (alliance->getName() == "BLUE")
    target_point = control::Point{144.0 - 68.5, 127.5};

  turnToPoint(target_point.getX(), target_point.getY(), target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 25.0);
  setElevatorVoltage(8.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  delay(75);
  target_velocity = 36.0;
  setGoToPointVelocity(target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1700, 0.5);
  waitForAllianceRing(500);
  armGoLoad();

  // turn to face alliance stake
  turnToAngle(M_PI / 2.0, target_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(M_PI / 2.0, 1000, M_PI / 25.0);

  target_velocity = 12.0;
  driveStraight(12.0, target_velocity, M_PI / 2.0);
  waitForDriveStraight(12.0, 500, 0.5);
  m_control_system->pause();

  delay(750);
  setElevatorVoltage(0.0);
  spinIntake(0.0);

  armGoAllianceStake();
  m_delayer->delay(600);

  target_velocity = 16.0;
  driveStraight(-16.0, target_velocity, M_PI / 2.0);
  delay(600);
  armGoNeutral();
  waitForDriveStraight(-16.0, 1000, 0.5);
  /*
    turnToAngle(3.0 * M_PI / 2.0, target_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(3.0 * M_PI / 2.0, 700, M_PI / 25.0);

    driveStraight(24.0, target_velocity, 3.0 * M_PI / 2.0);
    delay(1000);
    armGoAllianceStake();
    m_control_system->pause();

    */
  // display the runtime at the end
  current_time = getTime();
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "Runtime: %7.2f",
                      (current_time - start_time) / 1000.0);
}
}  // namespace auton
}  // namespace driftless