#ifndef __ODOMETRY_SUBSYSTEM_HPP__
#define __ODOMETRY_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/odometry/DistancePositionResetter.hpp"
#include "driftless/robot/subsystems/odometry/InertialPositionTracker.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for odometry code
/// @author Matthew Backman
namespace odometry {

/// @brief Class representing the odometry subsystem of the robot
/// @author Matthew Backman
class OdometrySubsystem : public ASubsystem {
 private:
  // the position tracker being used
  std::unique_ptr<IPositionTracker> m_position_tracker{};

  // the position resetter being used
  std::unique_ptr<IPositionResetter> m_position_resetter{};

 public:
  /// @brief Constructor
  /// @param position_tracker __std::unique_ptr<IPositionTracker>&__ The position tracker to use
  /// @param position_resetter __std::unique_ptr<IPositionResetter>&__ The position resetter to use
  OdometrySubsystem(std::unique_ptr<IPositionTracker>& position_tracker,
                    std::unique_ptr<IPositionResetter>& position_resetter);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Sends a command to the subsystem
  /// @param command_name __ESubsystemCommand__ The command to send
  /// @param args __va_list&__ The arguments for the command
  void command(ESubsystemCommand command_name, va_list& args) override;

  /// @brief Gets a specified state of the subsystem
  /// @param state_name __ESubsystemState__ The state to get
  /// @return __void*__ The state
  void* state(ESubsystemState state_name) override;
};

}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif