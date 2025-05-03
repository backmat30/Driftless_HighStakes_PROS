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

void OrangeRushAuton::ejectRingsLeft() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_REJECT_LEFT);
}

void OrangeRushAuton::ejectRingsRight() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_REJECT_RIGHT);
}

void OrangeRushAuton::ejectRingsForward() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_REJECT_FORWARD);
}

void OrangeRushAuton::pushIntakeOut() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_PUSH_OUT);
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

bool OrangeRushAuton::hasGoal() {
  bool has_goal{};
  void* has_goal_state{
      m_robot->getState(robot::subsystems::ESubsystem::CLAMP,
                        robot::subsystems::ESubsystemState::CLAMP_HAS_GOAL)};
  if (has_goal_state != nullptr) {
    has_goal = *static_cast<bool*>(has_goal_state);
  }
  return has_goal;
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
    setOdomPosition(105.0, 28.0, 3 * M_PI / 2.0);
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    setOdomPosition(144.0 - 105.0, 28.0, 3 * M_PI / 2.0);
  robot::subsystems::odometry::Position position{getOdomPosition()};
  double velocity{getOdomVelocity()};

  uint32_t current_time{start_time};
  control::Point target_point{};
  double target_distance{};
  double target_velocity{};
  double target_angular_velocity{4.0 * M_PI};

  startColorSort();
  pushIntakeOut();

  // Start the rush path
  std::vector<control::Point> rush_control_points{};
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    rush_control_points = std::vector<control::Point>{
        control::Point{105.0, 28.0}, control::Point{107.0, 30.0},
        control::Point{112.5, 42.0}, control::Point{114.85, 63.5}};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    rush_control_points =
        std::vector<control::Point>{control::Point{144.0 - 105.0, 28.0},
                                    control::Point{144.0 - 104.5, 34.0},
                                    control::Point{144.0 - 114.0, 42.0},
                                    control::Point{144.0 - 115.75, 61.5}};

  std::vector<control::Point> rush_path{
      control::path::BezierCurveInterpolation::calculate(rush_control_points)};
  target_point = rush_control_points.back();

  target_velocity = 83.0;

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
  target_velocity = 10.0;
  setFollowPathVelocity(target_velocity);
  while (target_distance > 5) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  waitForGoToPoint(target_point.getX(), target_point.getY(), 300, 1.0);

  setClamp(true);
  m_control_system->pause();
  delay(250);
  armGoNeutral();

  // Check if the goal was picked up
  if (!hasGoal()) {
    // Go to middle goal if the goal rush missed
    setClamp(false);

    turnToAngle(M_PI / 4.0, target_angular_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(M_PI / 4.0, 1200, M_PI / 50.0);

    // Set up the path to the middle goal
    std::vector<control::Point> middle_goal_control_points{};
    if (alliance->getAlliance() == alliance::EAlliance::RED)
      middle_goal_control_points = std::vector<control::Point>{
          control::Point{114.85, 63.25}, control::Point{112.0, 45.0},
          control::Point{86.0, 45.0}, control::Point{79.5, 64.5}};
    else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
      middle_goal_control_points =
          std::vector<control::Point>{control::Point{144.0 - 115.75, 61.5},
                                      control::Point{144.0 - 114.0, 70.0},
                                      control::Point{144.0 - 110.0, 80.0},
                                      control::Point{144.0 - 100.0, 90.0}};
    std::vector<control::Point> middle_goal_path{
        control::path::BezierCurveInterpolation::calculate(
            middle_goal_control_points)};

    target_velocity = 50.0;
    target_point = middle_goal_control_points.back();
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());

    followPath(middle_goal_path, target_velocity);

    while (target_distance > 8.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
    target_velocity = 10.0;
    setFollowPathVelocity(target_velocity);
    while (target_distance > 5) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
    goToPoint(target_point.getX(), target_point.getY(), target_velocity);

    waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);

    setClamp(true);
    m_control_system->pause();
    delay(250);

    target_velocity = 32.0;
    driveStraight(6.0, target_velocity, -M_PI / 6.0);
    waitForDriveStraight(6.0, 1000, 0.5);

    if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
      target_point = control::Point{47.5, 51.0};
    } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
      target_point = control::Point{144.0 - 51.0, 51.0};
    }

    turnToAngle(5.0 * M_PI / 6.0, target_angular_velocity,
                control::motion::ETurnDirection::CLOCKWISE);
    waitForTurnToAngle(5.0 * M_PI / 6.0, 1200, M_PI / 50.0);

    goToPoint(target_point.getX(), target_point.getY(), target_velocity);
    waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);
    m_control_system->pause();
  } else {
    // Set up the path under the ladder
    std::vector<control::Point> under_ladder_control_points{};
    if (alliance->getAlliance() == alliance::EAlliance::RED)
      under_ladder_control_points = std::vector<control::Point>{
          control::Point{114.85, 63.5}, control::Point{82.0, 52.0},
          control::Point{70.0, 62.0}, control::Point{51.0, 48.0}};
    else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
      under_ladder_control_points =
          std::vector<control::Point>{control::Point{144.0 - 24.0, 82.25},
                                      control::Point{144.0 - 46.0, 95.0},
                                      control::Point{144.0 - 72.0, 70.0},
                                      control::Point{144.0 - 90.0, 97.0}};

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
    target_velocity = 64.0;
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
    target_velocity = 20.0;
    setFollowPathVelocity(target_velocity);

    while (target_distance > 4.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
    ejectRingsRight();
    goToPoint(target_point.getX(), target_point.getY(), target_velocity);
    waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);
    m_control_system->pause();
  }

  // Wait for the blue robot to leave, robot is too zoomy
  current_time = getTime();
  waitForOpposingRing(2500);
  setIntakeVoltage(0.0);

  if (getTime() < current_time + 3250) {
    m_delayer->delayUntil(current_time + 3250);
  }

  if (!hasGoal()) {
    setClamp(false);
  } else {
    setElevatorVoltage(12.0);
    setIntakeVoltage(12.0);
    // go to ring by wall stake
    std::vector<control::Point> wall_stake_rings_control_points{};
    if (alliance->getAlliance() == alliance::EAlliance::RED)
      wall_stake_rings_control_points = std::vector<control::Point>{
          control::Point{46.0, 46.0}, control::Point{34.0, 2.0},
          control::Point{14.0, 30.0}, control::Point{11.5, 63.75}};
    else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
      wall_stake_rings_control_points =
          std::vector<control::Point>{control::Point{144.0 - 90.0, 97.0},
                                      control::Point{144.0 - 101.5, 112.0},
                                      control::Point{144.0 - 122.0, 102.5},
                                      control::Point{144.0 - 122.0, 78.0}};
    std::vector<control::Point> wall_stake_rings_path{
        control::path::BezierCurveInterpolation::calculate(
            wall_stake_rings_control_points)};
    target_point = control::Point{28.0, 22.0};
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());

    target_velocity = 48.0;
    setFollowPathVelocity(target_velocity);
    followPath(wall_stake_rings_path, target_velocity);

    while (target_distance > 18.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }

    target_velocity = 24.0;
    setFollowPathVelocity(target_velocity);

    while (target_distance > 10.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }

    while (target_distance < 16.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
    if (alliance->getAlliance() == alliance::EAlliance::RED) {
      setIntakeHeight(true);
    }

    ejectRingsLeft();

    target_velocity = 32.0;
    setFollowPathVelocity(target_velocity);
    target_point = wall_stake_rings_control_points.back();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());

    while (target_distance > 8.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
    target_velocity = 12.0;
    turnToPoint(target_point.getX(), target_point.getY(),
                target_angular_velocity, control::motion::ETurnDirection::AUTO);
    waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                       M_PI / 10.0);
    goToPoint(target_point.getX(), target_point.getY(), target_velocity);
    waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 0.5);

    position = getOdomPosition();

    m_control_system->pause();
    setIntakeHeight(false);
    waitForAllianceRing(1000);
    if (hasAllianceRing()) {
      setIntakeVoltage(-12.0);
    }
    delay(350);
  }

  // Go line up for corner
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{40.0, 40.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 110.0, 114.0};

  target_velocity = 83.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 2.0);

  // turn to corner
  setIntakeHeight(true);
  setIntakeVoltage(0.0);

  target_velocity = 64.0;
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{11.0, 10.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 10.0, 9.0};
  }
  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 25.0);

  ejectRingsRight();

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());

  while (target_distance > 18) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 20.0;
  setGoToPointVelocity(target_velocity);
  setIntakeVoltage(12.0);

  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 2.0);
  m_control_system->pause();

  setIntakeHeight(false);

  for (int i = 0; i < 3; ++i) {
    position = getOdomPosition();
    target_velocity = 20.0;
    driveStraight(12.0, target_velocity, position.theta);
    waitForDriveStraight(12.0, 1000, 0.5);

    delay(100);

    target_velocity = 40.0;
    driveStraight(-12.0, target_velocity, position.theta);
    waitForDriveStraight(-12.0, 1000, 0.5);

    delay(100);
  }

  // go to positive corner to be ready for op control

  target_velocity = 83.0;

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{122.0, 28.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 120.0, 32.0};
  }

  ejectRingsRight();

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                     M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 4000, 2.0);

  // display the runtime at the end
  current_time = getTime();
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 6, "Runtime: %7.2f",
                      (current_time - start_time) / 1000.0);
}
}  // namespace auton
}  // namespace driftless