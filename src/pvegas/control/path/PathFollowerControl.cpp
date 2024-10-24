#include "pvegas/control/path/PathFollowerControl.hpp"

namespace pvegas {
namespace control {
namespace path {
PathFollowerControl::PathFollowerControl(
    std::unique_ptr<pvegas::control::path::IPathFollower>& path_follower)
    : m_path_follower{std::move(path_follower)} {}

void PathFollowerControl::init() {
  if (m_path_follower) {
    m_path_follower->init();
  }
}

void PathFollowerControl::run() {
  if (m_path_follower) {
    m_path_follower->run();
  }
}

void PathFollowerControl::pause() {
  if (m_path_follower) {
    m_path_follower->pause();
  }
}

void PathFollowerControl::resume() {
  if (m_path_follower) {
    m_path_follower->resume();
  }
}

void PathFollowerControl::command(std::string command_name, va_list& args) {
  if (command_name == FOLLOW_PATH_COMMAND_NAME) {
    // get the robot from the va_list
    void* temp_robot{va_arg(args, void*)};
    // cast the robot to the desired type
    std::shared_ptr<pvegas::robot::Robot> robot{
        *static_cast<std::shared_ptr<pvegas::robot::Robot>*>(temp_robot)};
    // get the control path from the va_list
    void* temp_path{va_arg(args, void*)};
    // cast the path to the desired type
    std::vector<pvegas::control::Point> control_path{
        *static_cast<std::vector<pvegas::control::Point>*>(temp_path)};
    // get the max velocity from the va_list
    double velocity{va_arg(args, double)};

    // pass inputs to the follow path command
    m_path_follower->followPath(robot, control_path, velocity);
  } else if (command_name == SET_VELOCITY_COMMAND_NAME) {
    double velocity{va_arg(args, double)};
    m_path_follower->setVelocity(velocity);
  }
}

void* PathFollowerControl::state(std::string state_name) {
  void* result{nullptr};
  if (state_name == TARGET_REACHED_STATE_NAME) {
    result = new bool(m_path_follower->targetReached());
  }
  return result;
}
}  // namespace path
}  // namespace control
}  // namespace pvegas