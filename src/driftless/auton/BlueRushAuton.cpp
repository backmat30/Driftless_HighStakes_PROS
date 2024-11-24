#include "driftless/auton/BlueRushAuton.hpp"

#include "pros/screen.hpp"

namespace driftless {
namespace auton {
void BlueRushAuton::calibrateArm() {
  m_robot->sendCommand(ARM_SUBSYSTEM_NAME, ARM_CALIBRATE_COMMAND);
}

void BlueRushAuton::setClamp(bool clamped) {
  m_robot->sendCommand(CLAMP_SUBSYSTEM_NAME, CLAMP_SET_STATE_COMMAND, clamped);
}

void BlueRushAuton::setElevatorVoltage(double voltage) {
  m_robot->sendCommand(ELEVATOR_SUBSYSTEM_NAME, ELEVATOR_SET_VELOCITY, voltage);
}

void BlueRushAuton::updateRingSort(
    const std::shared_ptr<alliance::IAlliance>& alliance) {
  void* has_ring_state{
      m_robot->getState(RING_SORT_SUBSYSTEM_NAME, RING_SORT_HAS_RING_STATE)};
  bool has_ring{has_ring_state != nullptr &&
                *static_cast<bool*>(has_ring_state)};

  void* ring_rgb_state{
      m_robot->getState(RING_SORT_SUBSYSTEM_NAME, RING_SORT_GET_RGB_STATE)};
  io::RGBValue ring_rgb{*static_cast<io::RGBValue*>(ring_rgb_state)};

  void* position_state{
      m_robot->getState(ELEVATOR_SUBSYSTEM_NAME, ELEVATOR_POSITION_STATE)};
  double position{*static_cast<double*>(position_state)};

  void* distance_to_end_state{m_robot->getState(
      RING_SORT_SUBSYSTEM_NAME, RING_SORT_GET_DISTANCE_TO_END_STATE)};
  double distance_to_end{*static_cast<double*>(distance_to_end_state)};

  if (has_ring) {
    if ((alliance->getName() == "BLUE" && ring_rgb.red >= ring_rgb.blue) ||
        (alliance->getName() == "RED" && ring_rgb.blue >= ring_rgb.red)) {
      ring_sort_latest_ring_pos = position;
    }
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

void BlueRushAuton::spinIntake(double voltage) {
  m_robot->sendCommand(INTAKE_SUBSYSTEM_NAME, INTAKE_SPIN_COMMAND, voltage);
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
      MOTION_CONTROL_NAME, GO_TO_POINT_TARGET_REACHED))};
  return target_reached;
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

  // Set the robots starting values
  uint32_t start_time{getTime()};
  setOdomPosition(15.25, 125.0, M_PI / 2.0);
  robot::subsystems::odometry::Position position{getOdomPosition()};
  double velocity{getOdomVelocity()};

  control::Point target_point{};
  double target_distance{};
  double target_velocity{};

  // Start the rush path
  std::vector<control::Point> rush_control_points{
      control::Point{15.25, 115.0}, control::Point{15.0, 97.0},
      control::Point{15.0, 97.0}, control::Point{23.25, 84.5}};
  std::vector<control::Point> rush_path{
      control::path::BezierCurveInterpolation::calculate(rush_control_points)};
  target_point = rush_control_points.back();

  target_velocity = 48.0;

  followPath(rush_path, target_velocity);
  // Set up subsystems
  spinIntake(12.0);
  calibrateArm();
  m_delayer->delay(50);
  spinIntake(0.0);

  position = getOdomPosition();
  target_distance = distance(position.x, position.y, target_point.getX(),
                             target_point.getY());
  while (target_distance > 30.0) {
    m_delayer->delay(LOOP_DELAY);
    position = getOdomPosition();
    target_distance = distance(position.x, position.y, target_point.getX(),
                               target_point.getY());
  }

  goToPoint(target_point.getX(), target_point.getY(), target_velocity);
  waitForGoToPoint(target_point.getX(), target_point.getY(), 1000, 0.5);

  setClamp(true);
  m_control_system->pause();
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 5, "runtime: %7.2f",
                      ((m_clock->getTime() - start_time) / 1000.0));
  /*
      // start the intake/elevator
      spinIntake(12.0);
      setElevatorVoltage(12.0);

      // Move to the rings on the auton line
      std::vector<control::Point> gather_start_control_points{
          control::Point{125.5, 63.0}, control::Point{130.0, 60.0},
          control::Point{130.0, 60.0}, control::Point{125.5, 63.0}};
      std::vector<control::Point> gather_start_path{
          control::path::BezierCurveInterpolation::calculate(
              gather_start_control_points)};

      target_velocity = 12.0;
      followPath(gather_start_control_points, target_velocity);

      // run the color sorter
      while(!followPathTargetReached()) {
        updateRingSort(alliance);
        m_delayer->delay(LOOP_DELAY);
      }

      // Go along the gather path
      std::vector<control::Point> gather_control_points{
          control::Point{125.5, 63.0}, control::Point{120.5, 55.0},
          control::Point{114.0, 33.5}, control::Point{121.0, 24.0},
          control::Point{92.5, 13.0},  control::Point{85.0, 28.0},
          control::Point{72.5, 13.5},  control::Point{93.0, 6.5},
          control::Point{126.5, 18.5}, control::Point{136.0, 7.0}};
      std::vector<control::Point> gather_path{
          control::path::BezierCurveInterpolation::calculate(
              gather_control_points)};

      target_velocity = 12.0;
      followPath(gather_path, target_velocity);

      // Run the ring sorter while going along the path
      while(!followPathTargetReached()) {
        updateRingSort(alliance);
        m_delayer->delay(LOOP_DELAY);
      }
      */
}
}  // namespace auton
}  // namespace driftless