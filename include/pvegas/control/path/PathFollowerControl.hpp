#ifndef __PATH_FOLLOWER_CONTROL_HPP__
#define __PATH_FOLLOWER_CONTROL_HPP__

#include <memory>

#include "pvegas/control/AControl.hpp"
#include "pvegas/control/path/PIDPathFollowerBuilder.hpp"
namespace pvegas {
namespace control {
namespace path {
class PathFollowerControl : public pvegas::control::AControl {
 private:
  // name of the control
  static constexpr char CONTROL_NAME[]{"PATH FOLLOWING"};

  // name of the follow path command
  static constexpr char FOLLOW_PATH_COMMAND_NAME[]{"FOLLOW PATH"};

  // name of the set velocity command
  static constexpr char SET_VELOCITY_COMMAND_NAME[]{"SET VELOCITY"};

  // name of the target reached state
  static constexpr char TARGET_REACHED_STATE_NAME[]{"TARGET REACHED"};

  // path follower object
  std::unique_ptr<pvegas::control::path::IPathFollower> m_path_follower{};

 public:
  // constructor
  PathFollowerControl(
      std::unique_ptr<pvegas::control::path::IPathFollower>& path_follower);

  // initialize the control
  void init() override;

  // run the control
  void run() override;

  // pause the control
  void pause() override;

  // resume the control
  void resume() override;

  // send a command to the control
  void command(std::string command_name, va_list& args) override;

  // get a state of the control
  void* state(std::string state_name) override;
};
}  // namespace path
}  // namespace control
}  // namespace pvegas
#endif