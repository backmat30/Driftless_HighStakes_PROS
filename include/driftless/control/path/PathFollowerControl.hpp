#ifndef __PATH_FOLLOWER_CONTROL_HPP__
#define __PATH_FOLLOWER_CONTROL_HPP__

#include <memory>

#include "driftless/control/AControl.hpp"
#include "driftless/control/path/PIDPathFollowerBuilder.hpp"
namespace driftless {
namespace control {
namespace path {
class PathFollowerControl : public driftless::control::AControl {
 private:
  // path follower object
  std::unique_ptr<driftless::control::path::IPathFollower> m_path_follower{};

 public:
  // constructor
  PathFollowerControl(
      std::unique_ptr<driftless::control::path::IPathFollower>& path_follower);

  // initialize the control
  void init() override;

  // run the control
  void run() override;

  // pause the control
  void pause() override;

  // resume the control
  void resume() override;

  // send a command to the control
  void command(EControlCommand command_name, va_list& args) override;

  // get a state of the control
  void* state(EControlState state_name) override;
};
}  // namespace path
}  // namespace control
}  // namespace driftless
#endif