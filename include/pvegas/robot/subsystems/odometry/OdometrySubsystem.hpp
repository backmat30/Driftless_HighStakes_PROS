#ifndef __ODOMETRY_SUBSYSTEM_HPP__
#define __ODOMETRY_SUBSYSTEM_HPP__

#include <memory>

#include "pvegas/robot/subsystems/ASubsystem.hpp"
#include "pvegas/robot/subsystems/odometry/DistancePositionResetter.hpp"
#include "pvegas/robot/subsystems/odometry/InertialPositionTracker.hpp"

namespace pvegas {
namespace robot {
namespace subsystems {
namespace odometry {
class OdometrySubsystem : public ASubsystem {
 private:
  // name of the subsystem
  static constexpr char SUBSYSTEM_NAME[]{"ODOMETRY"};

  // --- COMMAND NAMES ---

  // command to set the position
  static constexpr char SET_POSITION_COMMAND_NAME[]{"SET POSITION"};

  // command to set the x position
  static constexpr char SET_X_COMMAND_NAME[]{"SET X"};

  // command to set the y position
  static constexpr char SET_Y_COMMAND_NAME[]{"SET Y"};

  // command to set the angle
  static constexpr char SET_THETA_COMMAND_NAME[]{"SET THETA"};

  // command to reset the x position
  static constexpr char RESET_X_COMMAND_NAME[]{"RESET X"};

  // command to reset the y position
  static constexpr char RESET_Y_COMMAND_NAME[]{"RESET Y"};

  // command to get the robot's position
  static constexpr char GET_POSITION_STATE_NAME[]{"GET POSITION"};

  // command to get the raw value of the position resetter
  static constexpr char GET_RESETTER_RAW_VALUE_STATE_NAME[]{
      "GET RESETTER RAW VALUE"};

  // the position tracker being used
  std::unique_ptr<IPositionTracker> m_position_tracker{};

  // the position resetter being used
  std::unique_ptr<IPositionResetter> m_position_resetter{};

 public:
  // constructor
  OdometrySubsystem(std::unique_ptr<IPositionTracker>& position_tracker,
                    std::unique_ptr<IPositionResetter>& position_resetter);

  // initialize the subsystem
  void init() override;

  // run the subsystem
  void run() override;

  // send a command to the subsystem
  void command(std::string command_name, va_list& args) override;

  // get a specified state of the subsystem
  void* state(std::string state_name) override;
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace pvegas
#endif