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

void BlueRushAuton::armGoLoad() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::ARM,
                       robot::subsystems::ESubsystemCommand::ARM_GO_LOAD);
}

void BlueRushAuton::armGoAllianceStake() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ARM,
      robot::subsystems::ESubsystemCommand::ARM_GO_ALLIANCE_STAKE);
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

void BlueRushAuton::ejectRingsLeft() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_REJECT_LEFT);
}

void BlueRushAuton::ejectRingsRight() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ELEVATOR,
      robot::subsystems::ESubsystemCommand::ELEVATOR_REJECT_RIGHT);
}

void BlueRushAuton::pushIntakeOut() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_PUSH_OUT);
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
          ring_rgb.red * 0.75 > ring_rgb.blue) ||
         (m_alliance->getAlliance() == alliance::EAlliance::BLUE &&
          ring_rgb.blue > ring_rgb.red * 0.85));
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
          ring_rgb.red * 0.95 < ring_rgb.blue) ||
         (m_alliance->getAlliance() == alliance::EAlliance::BLUE &&
          ring_rgb.blue < ring_rgb.red * 0.6));
  }

  return has_opposing_ring;
}

bool BlueRushAuton::hasGoal() {
  bool has_goal{};
  void* has_goal_state{
      m_robot->getState(robot::subsystems::ESubsystem::CLAMP,
                        robot::subsystems::ESubsystemState::CLAMP_HAS_GOAL)};
  if (has_goal_state != nullptr) {
    has_goal = *static_cast<bool*>(has_goal_state);
  }
  return has_goal;
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
    setOdomPosition(39.0, 19.5, 3 * M_PI / 2.0);
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    setOdomPosition(144.0 - 39.0, 19.5, 3 * M_PI / 2.0);
  robot::subsystems::odometry::Position position{getOdomPosition()};
  double velocity{getOdomVelocity()};

  uint32_t current_time{start_time};
  control::Point target_point{};
  double target_distance{};
  double target_velocity{};
  double target_angular_velocity{4.0 * M_PI};

  startColorSort();
  pushIntakeOut();
  calibrateArm();

  // Start the rush path
  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{28.5, 63.35};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 28.35, 63.35};

  target_velocity = 83.0;

  driveStraight(-72.0, target_velocity, 3.0 * M_PI / 2.0);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());

  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    while (target_distance > 36.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    while (target_distance > 36.0) {
      m_delayer->delay(LOOP_DELAY);
      position = getOdomPosition();
      target_distance = distance(position.x, position.y, target_point.getX(),
                                 target_point.getY());
    }
  }

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  while (target_distance > 12.5) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  target_velocity = 10.0;
  setGoToPointVelocity(target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 750, 1.0);

  setClamp(true);
  m_control_system->pause();

  armGoNeutral();

  position = getOdomPosition();
  target_velocity = 32.0;

  delay(100);
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    driveStraight(10.0, target_velocity, position.theta + (M_PI / 12.0));
    waitForDriveStraight(10.0, 1000, 0.5);
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    driveStraight(12.0, target_velocity, position.theta - (M_PI / 12.0));
    waitForDriveStraight(12.0, 1000, 0.5);
  }

  if (hasGoal()) {
    setElevatorVoltage(12.0);
    setIntakeVoltage(12.0);
    delay(50);

    // Go to first ring stack
    if (alliance->getAlliance() == alliance::EAlliance::RED) {
      target_point = control::Point{27.0, 49.5};
      turnToPoint(target_point.getX(), target_point.getY(),
                  target_angular_velocity / 3.0,
                  control::motion::ETurnDirection::CLOCKWISE);
    } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
      target_point = control::Point{144.0 - 27.0, 51.0};
      turnToPoint(target_point.getX(), target_point.getY(),
                  target_angular_velocity / 3.0,
                  control::motion::ETurnDirection::COUNTERCLOCKWISE);
    }

    waitForTurnToPoint(target_point.getX(), target_point.getY(), 750,
                       M_PI / 10.0);

    delay(400);

    setClamp(false);
    setElevatorVoltage(12.0);
    target_velocity = 24.0;
    goToPoint(target_point.getX(), target_point.getY(), target_velocity);
    waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 7.0);

    // wait until the robot sees an alliance ring to continue the path
    delay(250);
    position = getOdomPosition();
    target_velocity = 30.0;
    driveStraight(12.0, target_velocity, position.theta);
    waitForAllianceRing(1500);
    setElevatorVoltage(0.0);
    setIntakeVoltage(0.0);

    if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
      ejectRingsRight();
    } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
      ejectRingsRight();
    }

    position = getOdomPosition();
    turnToAngle(M_PI / 2.0, target_angular_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(M_PI / 2.0, 1000, M_PI / 10.0);
    // MISSED GOAL ROUTE
  } else {
    setClamp(false);
    position = getOdomPosition();

    target_velocity = 32.0;
    if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
      target_point = control::Point{38.0, 20.0};
    } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
      target_point = control::Point{144.0 - 38.0, 20.0};
    }

    goToPoint(target_point.getX(), target_point.getY(), target_velocity);
    waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 1.5);

    if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
      turnToAngle(M_PI, target_angular_velocity,
                  control::motion::ETurnDirection::AUTO);
      waitForTurnToAngle(M_PI, 750, M_PI / 5.0);
    } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
      turnToAngle(0.0, target_angular_velocity,
                  control::motion::ETurnDirection::AUTO);
      waitForTurnToAngle(0.0, 750, M_PI / 5.0);
    }
  }

  // go to the next mobile goal

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{52.0, 19.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 52.0, 20.0};
  }

  target_velocity = 83.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 2000, 1.0);

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{74.0, 23.25};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 74.0, 25.0};
  }

  target_velocity = 64.0;

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  // delay until close to the goal
  while (target_distance > 20.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }
  // slow down near the target
  target_velocity = 36.0;
  setGoToPointVelocity(target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 2.0);
  position = getOdomPosition();
  target_velocity = 24.0;
  driveStraight(-18.0, target_velocity, position.theta);
  delay(200);
  setClamp(true);
  waitForDriveStraight(-18.0, 3000, 1.0);
  m_control_system->pause();
  setElevatorVoltage(12.0);
  delay(75);

  target_velocity = 48.0;
  position = getOdomPosition();
  turnToAngle(-M_PI / 2.0, target_angular_velocity / 3.0,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(-M_PI / 2.0, 1000, M_PI / 4.0);

  // alliance stake
  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{71.0, 13.0};
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 73.5, 16.0};
  }

  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    ejectRingsRight();
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    ejectRingsLeft();
  }

  target_velocity = 72.0;

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1000,
                     M_PI / 10.0);
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  setElevatorVoltage(12.0);
  setIntakeVoltage(12.0);
  calibrateArm();

  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 1.0);
  m_control_system->pause();

  target_velocity = 32.0;
  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{71.5, 3.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 71.75, 3.0};
  }

  turnToAngle(-M_PI / 2.0, target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(-M_PI / 2.0, 1000, M_PI / 10.0);

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 550, 1.0);

  position = getOdomPosition();
  driveStraight(-1.0, target_velocity, position.theta);
  waitForDriveStraight(-1.0, 300, 0.5);
  m_control_system->pause();

  armGoAllianceStake();
  setElevatorVoltage(0.0);
  setIntakeVoltage(0.0);
  delay(700);

  target_velocity = 36.0;
  driveStraight(-15.0, target_velocity, -M_PI / 2.0);
  waitForDriveStraight(-15.0, 1250, 0.5);
  armGoNeutral();
  m_control_system->pause();

  // Orange preload

  setIntakeVoltage(12.0);
  setElevatorVoltage(12.0);

  if (alliance->getAlliance() == alliance::EAlliance::RED)
    target_point = control::Point{94.0, 17.0};
  else if (alliance->getAlliance() == alliance::EAlliance::BLUE)
    target_point = control::Point{144.0 - 94.0, 17.0};

  target_velocity = 83.0;
  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 5.0);
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 2.0);
  m_control_system->pause();

  // next rings

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{118.0, 48.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 121.0, 48.0};
  }

  target_velocity = 72.0;
  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 5.0);
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);

  setIntakeVoltage(12.0);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 24.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }

  target_velocity = 28.0;
  setGoToPointVelocity(target_velocity);

  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 2.0);

  position = getOdomPosition();
  target_velocity = 16.0;
  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    driveStraight(14.0, target_velocity, position.theta + M_PI / 10.0);
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    driveStraight(14.0, target_velocity, position.theta - M_PI / 10.0);
  }
  waitForAllianceRing(1000);
  setElevatorVoltage(0.0);
  m_control_system->pause();

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{120.0, 24.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 120.0, 24.0};
  }

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 5.0);
  setElevatorVoltage(12.0);

  target_velocity = 56.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 3000, 2.0);

  if (alliance->getAlliance() == alliance::EAlliance::RED) {
    target_point = control::Point{140.0, 0.0};
  } else if (alliance->getAlliance() == alliance::EAlliance::BLUE) {
    target_point = control::Point{144.0 - 140.0, 0.0};
  }

  turnToPoint(target_point.getX(), target_point.getY(), target_angular_velocity,
              control::motion::ETurnDirection::AUTO);
  waitForTurnToPoint(target_point.getX(), target_point.getY(), 1500,
                     M_PI / 25.0);

  position = getOdomPosition();

  setIntakeHeight(true);
  target_velocity = 24.0;
  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1250, 2.0);
  setIntakeHeight(false);
  waitForAllianceRing(1000);

  for (int i = 0; i < 2; ++i) {
    position = getOdomPosition();
    target_velocity = 30.0;
    driveStraight(-12.0, target_velocity, position.theta);
    waitForDriveStraight(-12.0, 800, 0.5);
    delay(150);

    target_velocity = 30.0;
    driveStraight(20.0, target_velocity, position.theta);
    waitForDriveStraight(20.0, 1200, 0.5);

    delay(150);
  }

  driveStraight(-20.0, target_velocity, position.theta);
  waitForDriveStraight(-20.0, 1000, 0.5);
  m_control_system->pause();
  delay(100);

  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    turnToAngle(3 * M_PI / 4.0, target_angular_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(3 * M_PI / 4.0, 1000, M_PI / 10.0);
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    turnToAngle(M_PI / 4.0, target_angular_velocity,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(M_PI / 4.0, 1000, M_PI / 10.0);
  }

  setClamp(false);
  setElevatorVoltage(-12.0);
  setIntakeVoltage(-12.0);

  if (m_alliance->getAlliance() == alliance::EAlliance::RED) {
    driveStraight(-16.0, target_velocity, 3.0 * M_PI / 4.0);
    waitForDriveStraight(-16.0, 1000, 0.5);

    position = getOdomPosition();
    target_velocity = 36.0;
    driveStraight(12.0, target_velocity, position.theta);
    waitForDriveStraight(12.0, 1000, 0.5);
    delay(100);

    target_point = control::Point{81.0, 56.0};
    turnToAngle(-M_PI / 4.0, target_angular_velocity / 2.0,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(-M_PI / 4.0, 1000, M_PI / 10.0);
  } else if (m_alliance->getAlliance() == alliance::EAlliance::BLUE) {
    driveStraight(-16.0, target_velocity, M_PI / 4.0);
    waitForDriveStraight(-16.0, 1000, 0.5);

    position = getOdomPosition();
    target_velocity = 36.0;
    driveStraight(12.0, target_velocity, position.theta);
    waitForDriveStraight(12.0, 1000, 0.5);
    delay(100);

    target_point = control::Point{144.0 - 74.0, 60.0};
    turnToAngle(-3.0 * M_PI / 4.0, target_angular_velocity / 2.0,
                control::motion::ETurnDirection::AUTO);
    waitForTurnToAngle(-3.0 * M_PI / 4.0, 1000, M_PI / 10.0);
  }
  delay(100);

  target_velocity = 54.0;
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
  target_velocity = 16.0;
  setGoToPointVelocity(target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1500, 1.0);
  m_control_system->pause();

  target_velocity = 72.0;
  goToPoint(144.0 - 48.0, 50.0, target_velocity);
  waitForGoToPoint(144 - 48, 50.0, 2000, 2.0);

  turnToAngle(M_PI / 2.0, target_angular_velocity, control::motion::ETurnDirection::AUTO);
  waitForTurnToAngle(M_PI / 2.0, 1000, M_PI / 5.0);
  m_control_system->pause();

  // Print the run-time for routing purposes, determine how much more can be
  // done after current tasks
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 5, "runtime: %7.2f",
                      ((m_clock->getTime() - start_time) / 1000.0));
}
}  // namespace auton
}  // namespace driftless