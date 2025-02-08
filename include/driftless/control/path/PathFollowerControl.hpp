#ifndef __PATH_FOLLOWER_CONTROL_HPP__
#define __PATH_FOLLOWER_CONTROL_HPP__

#include <memory>

#include "driftless/control/AControl.hpp"
#include "driftless/control/path/PIDPathFollowerBuilder.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for the path follower control
/// @author Matthew Backman
namespace path {

/// @brief Class representing the path follower control
/// @author Matthew Backman
class PathFollowerControl : public driftless::control::AControl {
 private:
  // path follower object
  std::unique_ptr<driftless::control::path::IPathFollower> m_path_follower{};

 public:
  /// @brief Constructs a new path follower control
  /// @param path_follower __std::unique_ptr<IPathFollower>& The path follower
  /// to control
  PathFollowerControl(
      std::unique_ptr<driftless::control::path::IPathFollower>& path_follower);

  /// @brief Initializes the path follower control
  void init() override;

  /// @brief Runs the path follower control
  void run() override;

  /// @brief Pauses the path follower control
  void pause() override;

  /// @brief Resumes the path follower control
  void resume() override;

  /// @brief Sends a command to the path follower
  /// @param command_name __EControlCommand__ The command being sent
  /// @param args __va_list&__ Potential arguements for the command
  void command(EControlCommand command_name, va_list& args) override;

  /// @brief Gets a state of the path follower
  /// @param state_name __EControlState__ The state to get
  /// @return __void*__ The state of the path follower
  void* state(EControlState state_name) override;
};
}  // namespace path
}  // namespace control
}  // namespace driftless
#endif