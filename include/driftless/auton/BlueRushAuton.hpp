#ifndef __BLUE_RUSH_AUTON_HPP__
#define __BLUE_RUSH_AUTON_HPP__

#include "driftless/auton/IAuton.hpp"
#include "driftless/control/Point.hpp"
#include "driftless/control/motion/ETurnDirection.hpp"
#include "driftless/control/path/BezierCurveInterpolation.hpp"
#include "driftless/control/path/IPathFollower.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

namespace driftless {
namespace auton {
class BlueRushAuton : public IAuton {
 private:
  static constexpr char AUTON_NAME[]{"BLUE_RUSH"};

  // MISC VALUES

  static constexpr uint8_t LOOP_DELAY{5};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::shared_ptr<robot::Robot> m_robot{};

  std::shared_ptr<control::ControlSystem> m_control_system{};

  std::shared_ptr<processes::ProcessSystem> m_process_system{};

  std::shared_ptr<alliance::IAlliance> m_alliance{};

  double ring_sort_latest_ring_pos{-__DBL_MAX__};

  void calibrateArm();

  void armGoNeutral();

  void setClamp(bool clamped);

  void setElevatorVoltage(double voltage);

  void updateRingSort();

  void waitForAllianceRing(uint32_t timeout);

  void waitForOpposingRing(uint32_t timeout);

  void spinIntake(double voltage);

  void setIntakeHeight(bool high);

  void setOdomPosition(double x, double y, double theta);

  void followPath(std::vector<control::Point>& path, double velocity);

  void setFollowPathVelocity(double velocity);

  void goToPoint(double x, double y, double velocity);

  void setGoToPointVelocity(double velocity);

  void waitForGoToPoint(double target_x, double target_y, uint32_t timeout,
                        double tolerance);

  void turnToPoint(double x, double y, double velocity,
                   control::motion::ETurnDirection direction);

  void waitForTurnToPoint(double x, double y, uint32_t timeout,
                          double tolerance);

  void turnToAngle(double theta, double velocity,
                   control::motion::ETurnDirection direction);

  void waitForTurnToAngle(double theta, uint32_t timeout, double tolerance);

  void driveStraight(double distance, double velocity, double theta);

  void waitForDriveStraight(double target_distance, uint32_t timeout,
                            double tolerance);

  void delay(uint32_t delay_time);

  uint32_t getTime();

  robot::subsystems::odometry::Position getOdomPosition();

  double getOdomVelocity();

  bool followPathTargetReached();

  bool goToPointTargetReached();

  bool turnTargetReached();

  bool driveStraightTargetReached();

  bool hasAllianceRing();

  bool hasOpposingRing();

 public:
  std::string getName() override;

  void init(std::shared_ptr<robot::Robot>& robot,
            std::shared_ptr<control::ControlSystem>& control_system,
            std::shared_ptr<driftless::processes::ProcessSystem>&
                process_system) override;

  void run(std::shared_ptr<driftless::robot::Robot>& robot,
           std::shared_ptr<driftless::control::ControlSystem>& control_system,
           std::shared_ptr<driftless::processes::ProcessSystem>& process_system,
           std::shared_ptr<driftless::alliance::IAlliance>& alliance,
           std::shared_ptr<rtos::IClock>& clock,
           std::unique_ptr<rtos::IDelayer>& delayer) override;
};
}  // namespace auton
}  // namespace driftless
#endif