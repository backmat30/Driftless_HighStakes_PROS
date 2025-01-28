#ifndef __ODOMETRY_SUBSYSTEM_HPP__
#define __ODOMETRY_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/odometry/DistancePositionResetter.hpp"
#include "driftless/robot/subsystems/odometry/InertialPositionTracker.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
class OdometrySubsystem : public ASubsystem {
 private:
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
  void command(ESubsystemCommand command_name, va_list& args) override;

  // get a specified state of the subsystem
  void* state(ESubsystemState state_name) override;
};
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif