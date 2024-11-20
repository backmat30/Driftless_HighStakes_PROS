#ifndef __BLUE_RUSH_AUTON_HPP__
#define __BLUE_RUSH_AUTON_HPP__

#include "driftless/auton/IAuton.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/path/BezierCurveInterpolation.hpp"
#include "driftless/control/path/IPathFollower.hpp"

namespace driftless {
namespace auton {
class BlueRushAuton : public IAuton {
 private:
  static constexpr char AUTON_NAME[]{"BLUE RUSH"};

  static constexpr char FOLLOW_PATH_COMMAND_NAME[]{"FOLLOW PATH"};

  static constexpr char PATH_FOLLOWER_CONTROL_NAME[]{"PATH FOLLOWING"};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::shared_ptr<robot::Robot> m_robot{};

  std::shared_ptr<control::ControlSystem> m_control_system{};

  std::vector<control::Point> m_rush_path{};

  void followPath(std::vector<control::Point>& path, double velocity);

 public:
  std::string getName() override;
  
  void init(std::shared_ptr<robot::Robot>& robot,
            std::shared_ptr<control::ControlSystem>& control_system) override;

  void run(std::shared_ptr<driftless::robot::Robot>& robot,
           std::shared_ptr<driftless::control::ControlSystem>& control_system,
           std::shared_ptr<driftless::alliance::IAlliance>& alliance,
           std::shared_ptr<rtos::IClock>& clock,
           std::unique_ptr<rtos::IDelayer>& delayer) override;
};
}  // namespace auton
}  // namespace driftless
#endif