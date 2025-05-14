#include "driftless/auton/OrangeMidRushAuton.hpp"

namespace driftless {
namespace auton {
void OrangeMidRushAuton::startColorSort() {
  m_process_system->sendCommand(
      processes::EProcess::AUTO_RING_REJECTION,
      processes::EProcessCommand::AUTO_RING_REJECTION_REJECT_RINGS, &m_robot,
      m_alliance);
}
void OrangeMidRushAuton::calibrateArm() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_CALIBRATE);
}

void OrangeMidRushAuton::armGoNeutral() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_NEUTRAL);
}

void OrangeMidRushAuton::armGoLoad() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
}

void OrangeMidRushAuton::armGoAllianceStake() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemCommand::ARM_GO_ALLIANCE_STAKE);
}

void OrangeMidRushAuton::setClamp(bool clamped) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::CLAMP,
                       robot::subsystems::ESubsystemCommand::CLAMP_SET_STATE,
                       clamped);
}

void OrangeMidRushAuton::setElevatorVoltage(double voltage) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_SET_VOLTAGE, voltage);
}

void OrangeMidRushAuton::waitForAllianceRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasAllianceRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void OrangeMidRushAuton::waitForOpposingRing(uint32_t timeout) {
  uint32_t current_time = getTime();
  uint32_t end_time = current_time + timeout;
  while (!hasOpposingRing() && current_time < end_time) {
    m_delayer->delay(LOOP_DELAY);
    current_time = getTime();
  }
}

void OrangeMidRushAuton::setIntakeVoltage(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_VOLTAGE,
                       voltage);
}

void OrangeMidRushAuton::setIntakeHeight(bool high) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_SET_HEIGHT,
                       high);
}

void OrangeMidRushAuton::ejectRingsLeft() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_REJECT_LEFT);
}

void OrangeMidRushAuton::ejectRingsRight() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_REJECT_RIGHT);
}

void OrangeMidRushAuton::pushIntakeOut() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_PUSH_OUT);
}

void OrangeMidRushAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemCommand::ODOMETRY_SET_POSITION, x, y, theta);
}

void OrangeMidRushAuton::followPath(std::vector<control::Point>& path,
                                    double velocity) {
  m_control_system->sendCommand(control::EControl::PATH_FOLLOWER,
                                control::EControlCommand::FOLLOW_PATH, &m_robot,
                                path, velocity);
}

void OrangeMidRushAuton::setFollowPathVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::PATH_FOLLOWER,
      control::EControlCommand::PATH_FOLLOWER_SET_VELOCITY, velocity);
}

void OrangeMidRushAuton::goToPoint(double x, double y, double velocity) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::GO_TO_POINT, &m_robot,
                                velocity, x, y);
}

void OrangeMidRushAuton::setGoToPointVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::EControlCommand::GO_TO_POINT_SET_VELOCITY, velocity);
}

void OrangeMidRushAuton::waitForGoToPoint(double target_x, double target_y,
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

void OrangeMidRushAuton::turnToPoint(
    double x, double y, double velocity,
    control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_POINT,
                                &m_robot, velocity, x, y, direction);
}

void OrangeMidRushAuton::waitForTurnToPoint(double x, double y,
                                            uint32_t timeout,
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

void OrangeMidRushAuton::turnToAngle(
    double theta, double velocity, control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_ANGLE,
                                &m_robot, velocity, theta, direction);
}

void OrangeMidRushAuton::waitForTurnToAngle(double theta, uint32_t timeout,
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

void OrangeMidRushAuton::driveStraight(double distance, double velocity,
                                       double theta) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::DRIVE_STRAIGHT,
                                &m_robot, velocity, distance, theta);
}

void OrangeMidRushAuton::waitForDriveStraight(double target_distance,
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

void OrangeMidRushAuton::delay(uint32_t delay_time) {
  uint32_t current_time{getTime()};
  uint32_t end_time{current_time + delay_time};

  while (current_time < end_time) {
    current_time = getTime();
    m_delayer->delay(LOOP_DELAY);
  }
}

uint32_t OrangeMidRushAuton::getTime() {
  uint32_t current_time{};
  if (m_clock) {
    current_time = m_clock->getTime();
  }
  return current_time;
}

robot::subsystems::odometry::Position OrangeMidRushAuton::getOdomPosition() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  return position;
}

double OrangeMidRushAuton::getOdomVelocity() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  double velocity{
      std::sqrt(std::pow(position.xV, 2) + std::pow(position.yV, 2))};
  return velocity;
}

bool OrangeMidRushAuton::followPathTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::PATH_FOLLOWER,
      control::EControlState::PATH_FOLLOWER_TARGET_REACHED))};
  return target_reached;
}

bool OrangeMidRushAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POINT_TARGET_REACHED))};
  return target_reached;
}

bool OrangeMidRushAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION, control::EControlState::TURN_TARGET_REACHED))};
  return target_reached;
}

bool OrangeMidRushAuton::driveStraightTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::DRIVE_STRAIGHT_TARGET_REACHED))};
  return target_reached;
}

bool OrangeMidRushAuton::hasAllianceRing() {
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
          ring_rgb.red * 0.75 > ring_rgb.blue) ||
         (m_alliance->getAlliance() == alliance::EAlliance::BLUE &&
          ring_rgb.blue > ring_rgb.red * 0.85));
  }

  return has_alliance_ring;
}

bool OrangeMidRushAuton::hasOpposingRing() {
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
          ring_rgb.red * 1.2 < ring_rgb.blue) ||
         (m_alliance->getAlliance() == alliance::EAlliance::BLUE &&
          ring_rgb.blue < ring_rgb.red * 0.6));
  }

  return has_opposing_ring;
}

bool OrangeMidRushAuton::hasGoal() {
  bool has_goal{};
  void* has_goal_state{
      m_robot->getState(robot::subsystems::ESubsystem::CLAMP,
                        robot::subsystems::ESubsystemState::CLAMP_HAS_GOAL)};
  if (has_goal_state != nullptr) {
    has_goal = *static_cast<bool*>(has_goal_state);
  }
  return has_goal;
}

bool OrangeMidRushAuton::isArmNeutral() {
  bool is_arm_neutral{*static_cast<bool*>(m_robot->getState(robot::subsystems::ESubsystem::ARM, robot::subsystems::ESubsystemState::ARM_IS_NEUTRAL))};
  return is_arm_neutral;
}

std::string OrangeMidRushAuton::getName() { return AUTON_NAME; }

void OrangeMidRushAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system,
    std::shared_ptr<driftless::processes::ProcessSystem>& process_system) {
  m_robot = robot;
  m_control_system = control_system;
  m_process_system = process_system;
}

void OrangeMidRushAuton::run(
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

  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{76.0, 63.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 84.0, 59.0};

  target_velocity = 83.0;
  
  driveStraight(-72.0, target_velocity, 3.0 * M_PI / 4.0);
  delay(100);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());

  while (target_distance > 4.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 36.0;
  setGoToPointVelocity(target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 750, 1.0);

  m_control_system->pause();
  delay(250);

  target_velocity = 40.0;
  if(m_alliance->getAlliance() == alliance::EAlliance::RED) {
  driveStraight(32.0, target_velocity, -M_PI / 6.0);
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
      driveStraight(32.0, target_velocity, -5.0 * M_PI / 6.0);
  }
  waitForDriveStraight(24.0, 2000, 1.0);
  calibrateArm();
  m_control_system->pause();

  while(!isArmNeutral()) {
    armGoNeutral();
    delay(LOOP_DELAY);
  }

  // Set up the path under the ladder
    std::vector<control::Point> under_ladder_control_points{};
    if (alliance->getAlliance() == alliance::EAlliance::RED)
      under_ladder_control_points = std::vector<control::Point>{
          control::Point{114.85, 63.5}, control::Point{82.0, 52.0},
          control::Point{70.0, 59.0}, control::Point{64.0, 54.0}};
    else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
      under_ladder_control_points =
          std::vector<control::Point>{control::Point{144.0 - 105.0, 50.0},
                                      control::Point{144.0 - 82.0, 62.0},
                                      control::Point{144.0 - 76.0, 62.0},
                                      control::Point{144.0 - 64.0, 54.0}};

    std::vector<control::Point> under_ladder_path{
        control::path::BezierCurveInterpolation::calculate(
            under_ladder_control_points)};

    // follow the path under the ladder
    target_point = under_ladder_control_points.back();
    target_velocity = 16.0;
    position = getOdomPosition();

    followPath(under_ladder_path, target_velocity);

    setIntakeVoltage(12.0);
    setElevatorVoltage(12.0);
    delay(100);

    position = getOdomPosition();
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

    while (target_distance > 15.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
    ejectRingsRight();
    goToPoint(target_point.getX(), target_point.getY(), target_velocity);
    waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 2.5);
    m_control_system->pause();

  // Wait for the blue robot to leave, robot is too zoomy
  current_time = getTime();
  setElevatorVoltage(0.0);

  m_delayer->delayUntil(start_time + 9000);
  setElevatorVoltage(12.0);

  // GRAB BLUE RUSH GOAL
  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{30.0, 61.0};

    turnToAngle(0.0, target_angular_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(0.0, 1000, M_PI / 5.0);
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 34.0, 61.0};

    turnToAngle(M_PI, target_angular_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(M_PI, 1000, M_PI / 5.0);
  }

  target_velocity = 24.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 1.5);

  setClamp(true);
  delay(100);

  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{49.5, 50.0};
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 49.5, 50.0};
  }

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1200, 0.5);
  m_control_system->pause();
  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);
  // go to ring by corner

  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{22.0, 15.0};
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 27.0, 22.0};
  }

  target_velocity = 83.0;
  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                     M_PI / 5.0);
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());

  while (target_distance > 16.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }

  target_velocity = 24.0;
  setGoToPointVelocity(target_velocity);

  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 1.0);
  delay(250);

  // Go to ring by wall stake

  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{18.0, 57.0};

    turnToAngle(3.0 * M_PI / 4.0, target_angular_velocity,
                control::motion::ETurnDirection::CLOCKWISE);
    waitForTurnToAngle(3.0 * M_PI / 4.0, 750, M_PI / 10.0);
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 22.5, 63.0};

    turnToAngle(M_PI / 4.0, target_angular_velocity,
                control::motion::ETurnDirection::COUNTERCLOCKWISE);
    waitForTurnToAngle(M_PI / 4.0, 750, M_PI / 10.0);
  }

  turnToPoint(target_point.getX(), target_point.getY(),
              target_angular_velocity / 2.5,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                     M_PI / 10.0);

  target_velocity = 83.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  ejectRingsLeft();

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());

  while (target_distance > 30.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    setIntakeHeight(true);
  }

  while (target_distance > 20.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }

  target_velocity = 18.0;
  setGoToPointVelocity(target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.5);

  m_control_system->pause();
  setIntakeHeight(false);
  waitForAllianceRing(1000);
  if (hasAllianceRing()) {
    setIntakeVoltage(-12.0);
  }
  delay(350);

  // Go line up for corner
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{25.0, 18.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 25.0, 29.0};
  }
  target_velocity = 60.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 2.0);

  // turn to corner
  setIntakeHeight(true);
  setIntakeVoltage(0.0);

  target_velocity = 36.0;
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{8.0, 0.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 4.0, 0.0};
  }
  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 25.0);

  ejectRingsRight();

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  setIntakeVoltage(12.0);

  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 2.0);
  m_control_system->pause();

  setIntakeHeight(false);

  for (int i = 0; i < 3; ++i) {
    position = getOdomPosition();
    target_velocity = 20.0;
    driveStraight(14.0, target_velocity, position.theta);
    waitForDriveStraight(14.0, 800, 0.5);

    delay(300);

    target_velocity = 30.0;
    driveStraight(-12.0, target_velocity, position.theta);
    waitForDriveStraight(-12.0, 800, 0.5);

    delay(100);
  }

  // go to positive corner to be ready for op control
  if (getTime() < start_time + 27000) {
    m_delayer->delayUntil(start_time + 27000);
  }

  target_velocity = 64.0;

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{122.0, 28.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 122.0, 28.0};
  }

  ejectRingsRight();

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                     M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 4000, 2.0);
  m_control_system->pause();

  // display the runtime at the end
  current_time = getTime();
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 6, "Runtime: %7.2f",
                      (current_time - start_time) / 1000.0);
}
}  // namespace auton
}  // namespace driftless