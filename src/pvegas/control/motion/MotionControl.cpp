#include "pvegas/control/motion/MotionControl.hpp"

namespace driftless {
namespace control {
namespace motion {
MotionControl::MotionControl(
    std::unique_ptr<driftless::control::motion::IDriveStraight>& drive_straight,
    std::unique_ptr<driftless::control::motion::IGoToPoint>& go_to_point,
    std::unique_ptr<driftless::control::motion::ITurn>& turn)
    : AControl{CONTROL_NAME},
      m_drive_straight{std::move(drive_straight)},
      m_go_to_point{std::move(go_to_point)},
      m_turn{std::move(turn)} {}

void MotionControl::init() {
  m_drive_straight->init();
  m_go_to_point->init();
  m_turn->init();
}

void MotionControl::run() {
  m_drive_straight->run();
  m_go_to_point->run();
  m_turn->run();
}

void MotionControl::pause() {
  switch (m_motion_type) {
    case EMotionType::DRIVE_STRAIGHT:
      m_drive_straight->pause();
      break;
    case EMotionType::GO_TO_POINT:
      m_go_to_point->pause();
      break;
    case EMotionType::TURN:
      m_turn->pause();
      break;
  }
}

void MotionControl::resume() {
  switch (m_motion_type) {
    case EMotionType::DRIVE_STRAIGHT:
      m_drive_straight->resume();
      break;
    case EMotionType::GO_TO_POINT:
      m_go_to_point->resume();
      break;
    case EMotionType::TURN:
      m_turn->resume();
      break;
  }
}

void MotionControl::command(std::string command_name, va_list& args) {
  if (command_name == DRIVE_STRAIGHT_COMMAND_NAME) {
    if (m_motion_type != EMotionType::DRIVE_STRAIGHT) {
      pause();
      m_motion_type = EMotionType::DRIVE_STRAIGHT;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    double distance{va_arg(args, double)};
    double theta{va_arg(args, double)};

    m_drive_straight->driveStraight(robot, velocity, distance, theta);

  } else if (command_name == GO_TO_POINT_COMMAND_NAME) {
    if (m_motion_type != EMotionType::GO_TO_POINT) {
      pause();
      m_motion_type = EMotionType::GO_TO_POINT;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    Point point{*va_arg(args, Point*)};

    m_go_to_point->goToPoint(robot, velocity, point);

  } else if (command_name == TURN_TO_ANGLE_COMMAND_NAME) {
    if (m_motion_type != EMotionType::TURN) {
      pause();
      m_motion_type = EMotionType::TURN;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    double theta{va_arg(args, double)};
    ETurnDirection direction{va_arg(args, ETurnDirection)};

    m_turn->turnToAngle(robot, velocity, theta, direction);

  } else if (command_name == TURN_TO_POINT_COMMAND_NAME) {
    if (m_motion_type != EMotionType::TURN) {
      pause();
      m_motion_type = EMotionType::TURN;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    Point point{*va_arg(args, Point*)};
    ETurnDirection direction{va_arg(args, ETurnDirection)};

    m_turn->turnToPoint(robot, velocity, point, direction);

  } else if (command_name == SET_DRIVE_STRAIGHT_VELOCITY_COMMAND_NAME) {
    double velocity{va_arg(args, double)};
    m_drive_straight->setVelocity(velocity);

  } else if (command_name == SET_GO_TO_POINT_VELOCITY_COMMAND_NAME) {
    double velocity{va_arg(args, double)};
    m_go_to_point->setVelocity(velocity);
  }
}

void* MotionControl::state(std::string state_name) {
  void* result{};
  if (state_name == DRIVE_STRAIGHT_TARGET_REACHED_STATE_NAME) {
    result = new bool{m_drive_straight->targetReached()};
  } else if (state_name == GO_TO_POINT_TARGET_REACHED_STATE_NAME) {
    result = new bool{m_go_to_point->targetReached()};
  } else if (state_name == TURN_TARGET_REACHED_STATE_NAME) {
    result = new bool{m_turn->targetReached()};
  }

  return result;
}
}  // namespace motion
}  // namespace control
}  // namespace pvegas